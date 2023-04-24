# Script to automate the process of archiving build artifacts for pool tests
# Useful to save in order to revert to previous pool tests in case the code is acting buggy

if [ -z $1 ]; then
    echo "Usage: $0 [dir]"
    exit 1
fi

firmware_repo=../titan_firmware

if ! [ -d "$firmware_repo" ]; then
    # Note this is set because I keep the firmware archive in a folder outside the titan_firmware repo
    # Whoever is building the firmware for relases can move this to whatever folder is recommended
    echo "Unable to locate firmware repo at '$firmware_repo'"
    echo "Please configure the firmware_repo variable in this script to point to the correct folder"
    exit 1
fi

dest_dir=$(realpath "$1")

pushd "$firmware_repo" > /dev/null
if ! (git diff --quiet --exit-code && test -z "$(git ls-files $(git rev-parse --show-toplevel) --exclude-standard --others)"); then
    echo "Git repo has uncomitted changes"
    read -p "Continue build/archive? [y/N]: " -n 1 -r
    echo

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi
popd > /dev/null

set -e

images="talos/CameraCageBB talos/ESCBoard talos/PowerBoard talos/SmartBattery"
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
    make -j
    popd > /dev/null
    popd > /dev/null
done

echo
echo "==================="
echo "Archiving..."
echo "==================="

mkdir -p "$1"

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

touch "$dest_dir/full_ota.tar"
find "$dest_dir" -name '*_ota.uf2' -type f -execdir tar rf "$dest_dir/full_ota.tar" {} +
