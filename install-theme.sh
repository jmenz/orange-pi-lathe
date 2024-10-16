#!/bin/bash

if [ -d /usr/share/themes/touchy-manulathe ]; then
  rm -r /usr/share/themes/touchy-manulathe
fi

ln -s "`pwd`"/theme /usr/share/themes/touchy-manulathe
