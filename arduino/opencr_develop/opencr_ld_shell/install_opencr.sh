#! /bin/bash

echo "Install OpenCR Update V171016"

if (($#==2))
then
  mkdir opencr_update
  tar -xvf opencr_update.tar.bz2 -C opencr_update
  cd ./opencr_update
else
  echo "wrong parameter "
  echo "install_opencr.sh <port> burger"
  echo "install_opencr.sh <port> waffle"
fi

exit
