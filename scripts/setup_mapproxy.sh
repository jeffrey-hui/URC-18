#!/usr/bin/env bash

set -e 

echo "[*] Welcome to the MapProxy setup wizard."
echo "[!] Running sudo to get cache ready..."

sudo true

echo "[*] Root is ok."
echo "[!] Do you want to install MapProxy? Press Ctrl+C to stop, or any other key to continue"
read


echo "[*] Continuing"
echo "[!] Checking if docker is installable"
if sudo apt-get install -y docker-ce; then
	echo "[*] Docker has been installed"
else
	echo "[!] Adding apt repos... [!]"
	sudo apt-get install -y \
		    apt-transport-https \
		        ca-certificates \
			    curl \
			        software-properties-common
	curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
	sudo add-apt-repository -y \
		   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
		      $(lsb_release -cs) \
		         stable"
	echo "[!] Updating [!]"
	sudo apt-get update
	echo "[!] Installing docker [!]"
	sudo apt-get install -y docker-ce
	echo "[!] Adding your user to the docker group"
	sudo adduser $USER docker
	echo "[*] Done."
fi

if ! sudo docker container ls | grep mapviz_proxy; then
	echo "[!] Creating container [!]"
	sudo docker create --name mapviz_proxy -v ~/mapproxy:/mapproxy -p 8080:8080 -t danielsnider/mapproxy
	echo "[*] Done."
else
	echo "[*] Container exists."
fi

echo "[*] Docker and mapproxy are now setup. The proxy_node node will start and stop things as needed. Make sure to re-log for groups to take effect"
echo "[*] Done."

