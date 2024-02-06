# Script to automate the process of archiving build artifacts for pool tests
# Useful to save in order to revert to previous pool tests in case the code is acting buggy

if ! cd "$(realpath "$(dirname $0)")"; then
    echo "Failed to navigate to script directory"
    exit 1
fi

firmware_repo="$(realpath ../titan_firmware)"

if ! [ -d "$firmware_repo" ]; then
    # Note this is set because I keep the firmware archive in a folder outside the titan_firmware repo
    # Whoever is building the firmware for relases can move this to whatever folder is recommended
    echo "Unable to locate firmware repo at '$firmware_repo'"
    echo "Ensure you followed the steps to symlink archive.sh to the firmware_archive folder"
    exit 1
fi

pushd "$firmware_repo" > /dev/null
if ! (git diff --quiet --exit-code && test -z "$(git ls-files $(git rev-parse --show-toplevel) --exclude-standard --others)"); then
    echo "Git repo has uncomitted changes"
    read -p "Continue build/archive? [y/N]: " -n 1 -r
    echo

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    read -p "Would you like to tag and push your commit? [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        while true;
        do
            read -p "Please enter in the name for the tag (ie. pool-test-2023-08-25): "
            tag_name=$REPLY
            if ! [[ $tag_name =~ ^[a-z0-9\-]{1,30}$ ]]; then
                echo "Name must only contain lowercase characters, numbers, and dashes (less than 30 characters)"
                echo
                continue
            fi

            popd > /dev/null
            dest_dir=$(realpath "$tag_name")
            pushd "$firmware_repo" > /dev/null

            if [ -d "$dest_dir" ]; then
                echo "Error: Directory '$dest_dir' already exists"
                echo
                continue
            fi

            break
        done

        read -p "Please type in your tag message (i.e. Pool Test on 8/25/2023): "
        tag_message=$REPLY

        if ! git push; then
            exit 1
        fi
        if ! git tag -a $tag_name -m "$tag_message"; then
            exit 1
        fi
        git push origin $tag_name
    fi
fi
popd > /dev/null

if [ -z "$dest_dir" ]; then
    while true;
    do
        read -p "Please enter in a directory name for the firmware archive folder: "

        if ! [[ $REPLY =~ ^[a-z0-9\-]{1,30}$ ]]; then
            echo "Directory may only contain lowercase characters, numbers, and dashes (less than 30 characters)"
            echo "$REPLY"
            continue
        fi

        dest_dir=$(realpath "$REPLY")

        if [ -d "$dest_dir" ]; then
                echo "Error: Directory '$dest_dir' already exists"
                echo
                continue
        fi

        break
    done
fi

set -e

images="talos/Actuator talos/CameraCageBB talos/ESCBoard talos/PowerBoard talos/SmartBattery puddles/Backplane"
image_types=".elf _ota.uf2 _with_bl.uf2"

for target in $images; do
    echo
    echo "==================="
    echo "Building $target"
    echo "==================="
    pushd "$firmware_repo/$target" > /dev/null
    rm -rf build
    mkdir build
    pushd build > /dev/null
    cmake ..
    make -j$(nproc)
    popd > /dev/null
    popd > /dev/null
done

echo
echo "==================="
echo "Archiving..."
echo "==================="

mkdir -p "$dest_dir"

for target in $images; do
    target_set=0
    build_dir=$firmware_repo/$target/build/
    for type in $image_types; do
        if [[ $(find "$build_dir" -maxdepth 1 -type f -name "*$type" -printf '.' | wc -c) != 1 ]]; then
            echo "No/multiple build targets found matching *$type for $target"
            exit 1
        fi
        targetpath=$(find "$build_dir" -maxdepth 1 -type f -name "*$type")

        if [[ $target_set == 0 ]]; then
            targetname=$(basename "$targetpath" "$type")
            mkdir -p "$dest_dir/$targetname"
            target_set=1
            echo "Archiving build targets for $targetname"
        elif [[ "$targetname" != $(basename "$targetpath" "$type") ]]; then
            echo "Unexpected file $targetpath found in $targetname"
            exit 1
        fi

        destname=$(basename "$targetpath")

        if [ -f "$dest_dir/$targetname/$destname" ]; then
            echo "Multiple files archived with the same name: $dest_dir/$targetname/$destname"
            exit 1
        fi
        cp "$targetpath" "$dest_dir/$targetname/$destname"
    done
done

echo "Creating ota archive for orin deployment"
touch "$dest_dir/full_ota.tar"
find "$dest_dir" -name '*_ota.uf2' -type f -execdir tar rf "$dest_dir/full_ota.tar" {} +

echo "Creating firmware archive for git release"
tar -C "$dest_dir" -zcf "$dest_dir"/firmware_archive.tgz $(find $dest_dir -maxdepth 1 -mindepth 1 -type d -printf '%f\n')

micro_ros_pico_folder=$firmware_repo/lib/micro_ros_pico/
echo "Creating libmicroros archive for git release"
tar -C "$micro_ros_pico_folder" -zcf "$dest_dir"/libmicroros_build.tgz libmicroros available_ros2_types built_packages

echo
read -p "Would you like to upload firmware to the Orin? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    remote_firmware_deploy_path="~/firmware_deploy"

    dest_dir_name="$(basename -- $dest_dir)"
    ssh ros@orin mkdir $remote_firmware_deploy_path/$dest_dir_name
    scp $dest_dir/full_ota.tar ros@orin:$remote_firmware_deploy_path/$dest_dir_name
    ssh ros@orin tar -C "$remote_firmware_deploy_path/$dest_dir_name" -xvf $remote_firmware_deploy_path/$dest_dir_name/full_ota.tar

    echo
    read -p "Would you like to flash the firmware to the robot? [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        ssh ros@orin $remote_firmware_deploy_path/bin/upload_tool $remote_firmware_deploy_path/$dest_dir_name/power_board_ota.uf2
        ssh ros@orin $remote_firmware_deploy_path/bin/upload_tool $remote_firmware_deploy_path/$dest_dir_name/actuator_mk2__ota.uf2
        ssh ros@orin $remote_firmware_deploy_path/bin/upload_tool $remote_firmware_deploy_path/$dest_dir_name/camera_cage_bb_ota.uf2
        ssh ros@orin $remote_firmware_deploy_path/bin/upload_tool $remote_firmware_deploy_path/$dest_dir_name/esc_board_ota.uf2
        ssh ros@orin $remote_firmware_deploy_path/bin/upload_tool $remote_firmware_deploy_path/$dest_dir_name/esc_board_ota.uf2
    fi
fi
