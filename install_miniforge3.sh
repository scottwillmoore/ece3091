pushd ~
curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh
sh Miniforge3-$(uname)-$(uname -m).sh -b
./miniforge3/bin/conda init
popd
