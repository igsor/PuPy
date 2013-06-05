#!/bin/bash

usage()
{
cat << EOF
usage: $0 -c <puppy controller> -s <supervisor controller> [optional options] <output>

   Note: It may be convenient to set the default value of -b in
         this script manually (default is script location)

OPTIONS:
   -c      Puppy controller
   -s      Supervisor controller
   -t      Terrain
   -p      Puppy
   -f      Overwrite target
   -b      Directory of your webots data
   -h      Show this message
EOF
}

WEBOTS="/usr/local/bin/webots"
BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/data"
TRG=

PCTRL=
SCTRL=
WORLD="default"
PUPPY="default"
SUPERVISOR="default"
FORCE=
while getopts “hfp:s:t:c:b:” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         f)
             FORCE=1
             ;;
         p)
             PUPPY=$OPTARG
             ;;
         t)
             WORLD=$OPTARG
             ;;
         c)
             PCTRL=$OPTARG
             ;;
         s)
             SCTRL=$OPTARG
             ;;
         b)
             BASE=$OPTARG
             ;;
         ?)
             usage
             exit
             ;;
     esac
done

shift $((OPTIND-1))

# target
if [[ -z $@ ]]; then
    usage
    exit 1
fi

TRG=$@

if [[ -e $TRG ]]; then
    if [[ -z $FORCE ]]; then
        echo "Target exists. Use -f to overwrite"
    else
        rm -R $TRG
    fi
fi

# check input files
if [[ -z $PCTRL ]] || [[ -z $SCTRL ]] || [[ -z $TRG ]]
then
     usage
     exit 1
fi

# check working directory
if [[ ! -e $BASE/terrain ]] || [[ ! -e $BASE/puppy ]] || [[ ! -e $BASE/supervisor ]] || [[ ! -e $BASE/supplementary ]]
then
    echo "Base directory seems not to fit. Please change your working directory"
    echo "The current base is $BASE"
    usage
    exit 1
fi

# check other input files
if [[ ! -e $BASE/terrain/$WORLD ]]; then echo "WORLD not found"; exit 1; fi
if [[ ! -e $BASE/puppy/$PUPPY ]]; then echo "PUPPY not found"; exit 1; fi
if [[ ! -e $BASE/supervisor/$SUPERVISOR ]]; then echo "SUPERVISOR not found"; exit 1; fi

# create data structure
mkdir -p $TRG
cp -R $BASE/supplementary/* $TRG
mkdir -p $TRG/controllers/

# set up world file
cat $BASE/terrain/$WORLD > $TRG/worlds/puppy_world.wbt
cat $BASE/puppy/$PUPPY | sed 's/controller ".*"/controller "puppyController"/' >> $TRG/worlds/puppy_world.wbt
cat $BASE/supervisor/$SUPERVISOR | sed 's/controller ".*"/controller "supervisorController"/' >> $TRG/worlds/puppy_world.wbt

# copy controller
mkdir -p $TRG/controllers/puppyController
mkdir -p $TRG/controllers/supervisorController
PPTH=`readlink -f $PCTRL`
SPTH=`readlink -f $SCTRL`
#cat $BASE/controllers/generic/generic.py | sed "s:^CMD='':CMD='$PPTH':" > $TRG/controllers/puppyController/puppyController.py
#cat $BASE/controllers/generic/generic.py | sed "s:^CMD='':CMD='$SPTH':" > $TRG/controllers/supervisorController/supervisorController.py
ln -s $PPTH $TRG/controllers/puppyController/puppyController.py
ln -s $SPTH $TRG/controllers/supervisorController/supervisorController.py


# start webots
cd $TRG
$WEBOTS worlds/puppy_world.wbt
