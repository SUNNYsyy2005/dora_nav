#!/bin/bash

dirs=("amcl" "imu" "lidar" "teb")

for dir in "${dirs[@]}"; do
    echo "Processing directory: $dir"

    if [ -d "build/$dir/build" ]; then
        echo "Removing $dir/build directory..."
        rm -rf "build/$dir/build"
    fi

    if [ -f "build/$dir/CMakeLists.txt" ]; then
        echo "Rebuilding $dir..."
        mkdir -p "build/$dir/build"
        cd "build/$dir/build"
        cmake ..
        make
        cd -
    else
        echo "Warning: CMakeLists.txt not found in $dir, skipping build."
    fi
done

if [ -f "control/Cargo.toml" ]; then
    echo "Building control project with Cargo..."
    cd control
    cargo build
    cd -
else
    echo "Warning: Cargo.toml not found in control directory, skipping Cargo build."
fi

echo "All operations completed."

