sudo kill -9 $(sudo lsof -t -i:$1)
#  sudo lsof -i -P -n | grep LISTEN
