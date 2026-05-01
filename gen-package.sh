#! /bin/bash -e
export DESTDIR=build-temp
rm -rf $DESTDIR
mkdir $DESTDIR

# create dockcross script
echo "Generating dockercross script"
IMG=tvoneicken/sensorgnome-dockcross:armv7-rpi-bookworm-main
docker run $IMG >sensorgnome-dockcross
chmod +x sensorgnome-dockcross

echo "Cross-compiling and installing"
make clean
./sensorgnome-dockcross -i $IMG \
    make -j4 install DESTDIR=$DESTDIR STRIP=armv7-unknown-linux-gnueabi-strip

# Boilerplate package generation
cp -r DEBIAN $DESTDIR
sed -e "/^Version/s/:.*/: $(date +%Y.%j)/" -i $DESTDIR/DEBIAN/control # set version: YYYY.DDD
mkdir -p packages
dpkg-deb --root-owner-group --build $DESTDIR packages
# dpkg-deb --contents packages
ls -lh packages
