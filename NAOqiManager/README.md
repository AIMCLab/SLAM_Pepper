# NAOqi Manager

Data access manager of Softbank Pepper robot.

## Environment

- CMake

  ```sh
  sudo apt-get install cmake
  ```

- qiBuild

  ```sh
  pip install qibuild
  ```

## Configure SDK

> See http://doc.aldebaran.com/2-5/dev/cpp/install_guide.html for details.

1. Download and extract the [NAOqi C++ SDK](https://community.ald.softbankrobotics.com/en/resources/software/pepper-sdks-and-documentation-255).

2. Creating your toolchain

   ```sh
   qitoolchain create mytoolchain /path/to/naoqi-sdk/toolchain.xml
   ```

3. Creating your configuration

   ```sh
   cd /path/to/myWorktree
   qibuild init
   qibuild add-config myconfig -t mytoolchain --default
   ```

## Build and Run

```sh
make config
make
make run [ip=xxx.xxx.xxx.xxx]
```
