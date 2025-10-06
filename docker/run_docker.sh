xhost +local:root

docker run -it --rm \
  --name yumi_cart_container \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/YUMI_CART:/workspace/YUMI_CART \
  junspring/yumi-cart:latest
