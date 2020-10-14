#! /bin/bash


architecture=""
case $(uname -m) in
    i386)    architecture="386" ;;
    i686)    architecture="386" ;;
    x86_64)  architecture="amd64" ;;
    armv7l)  architecture="arm" ;;
    aarch64) architecture="arm" ;;
    arm)     dpkg --print-architecture | grep -q "arm64" && architecture="arm64" || architecture="arm" ;;
esac

echo $(uname -m)
echo $architecture

shell_cmd=""
if [ "$architecture" == "arm" ]
then
  shell_cmd="./opencr_ld_shell_arm"
else
  shell_cmd="./opencr_ld_shell_x86"
fi

echo "OpenCR Update Start.."
if (($#==2))
then
  $shell_cmd $1 115200 $2 1
else
  echo "wrong parameter "
  echo "update.sh <port> fw_name"
fi

exit
