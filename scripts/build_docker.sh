FILE_DIR=$(realpath $(dirname $0))

docker build -f $FILE_DIR/../Docker/Dockerfile -t middleware_test:humble  .