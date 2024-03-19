#!/bin/bash

FAILED=0
SUMMARY=""

run_test () {
  printf "\n\n-------- Running tests for $1 --------\n"
  cd $1
  mkdir build
  cd build
  cmake ..
  make
  if ./"$2"; then
    echo "Test passed!"
    summary+="$1 - PASSED"
  else
    exit_code=$?
    echo "Test failed with exit code $exit_code"
    FAILED=1
    summary+="$1 - FAILED (with exit code $exit_code)"
  fi
  summary+=$'\n'

  cd ../..
}

# run_test "directory_name" "executable_name"
run_test "titan_list" "titan_list_tests"
run_test "titan_queue" "titan_queue_tests"

printf "\n\n-------- Summary --------\n"
printf "$summary"
if [ "$FAILED" -eq "0" ]; then
  echo "Test suite passed!"
else
  echo "Test suite failed!"
  exit 1
fi
