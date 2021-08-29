pushd ~
curl -L https://starship.rs/install.sh -o Starship-$(uname)-$(uname -m).sh
mkdir -p ./bin
sh ~/Starship-$(uname)-$(uname -m).sh -y -b ./bin
popd
