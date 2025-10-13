xhost +local:root

docker run -it --rm \
  --name yumi_cart_container \
  --net=host \
  --ipc=host \
  --privileged \
  --device /dev/ttyUSB0:/dev/ttyRplidar \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/YUMI_CART:/yumicart_ws \
  junspring/yumi-cart:latest
