#!/bin/bash

# TODO: Copy stuff like textures, prototypes, ...

usage()
{
cat << EOF
usage: $0 options <Input world file>

   <DESC>

OPTIONS:
   -a      Save terrain, puppy, supervisor (give name)
   -t      Save terrain (give name)
   -p      Save puppy (give name)
   -s      Save supervisor (give name)
   -c      Save controllers (give path)
   -b      Target base directory (give path)
   -h      Show this message
EOF
}

SAVECONTROLLER=0
BASE="/home/matthias/studium/master/work/webots/data"
INPUT=
TERRAIN=
PUPPY=
SUPERVISOR=
CTRL=

while getopts “a:t:p:s:c:b:h” OPTION
do
     case $OPTION in
         a)
             TERRAIN=$OPTARG
             PUPPY=$OPTARG
             SUPERVISOR=$OPTARG
             ;;
         t)
             TERRAIN=$OPTARG
             ;;
         p)
             PUPPY=$OPTARG
             ;;
         s)
             SUPERVISOR=$OPTARG
             ;;
         c)
             CTRL=$OPTARG
             ;;
         b)
             BASE=$OPTARG
             ;;
         h)
             usage
             exit 1
             ;;
         ?)
             echo "?"
             usage
             exit
             ;;
     esac
done

shift $((OPTIND-1))
INPUT=$@

# check input
if [[ ! -e $INPUT ]] || [[ -z $BASE ]]
then
     usage
     exit 1
fi

# analyze file
START_PUPPY=`cat $INPUT | grep -n '^DEF puppy Robot' | awk '{split($0,a,":"); print a[1]}'`
START_SUPER=`cat $INPUT | grep -n '^Supervisor' | awk '{split($0,a,":"); print a[1]}'`
LINES=`wc -l $INPUT | awk '{split($0,a); print a[1]}'`

if [ $START_PUPPY -gt $START_SUPER ]; then
    echo "Cannot split the world file; Please reformat"
    exit 1
fi

HEAD=`head -n $[$START_PUPPY-1] $INPUT`
ROBOT=`head -n $[$START_SUPER-1] $INPUT | tail -n $[$START_SUPER - $START_PUPPY]`
SUPER=`tail -n $[$LINES-$START_SUPER+1] $INPUT`

mkdir -p $BASE/terrain $BASE/puppy $BASE/supervisor

if [[ ! -z $TERRAIN ]]; then echo "$HEAD" > $BASE/terrain/$TERRAIN; fi
if [[ ! -z $PUPPY ]]; then echo "$ROBOT" > $BASE/puppy/$PUPPY; fi
if [[ ! -z $SUPERVISOR ]]; then echo "$SUPER" > $BASE/supervisor/$SUPERVISOR; fi

if [[ $CTRL ]]
then
    PCTRL=`echo "$ROBOT" | grep 'controller' | sed 's/^.*controller\s\+"\(.*\)"/\1/'`
    SCTRL=`echo "$SUPER" | grep 'controller' | sed 's/^.*controller\s\+"\(.*\)"/\1/'`
    
    PTH=`readlink -f $INPUT`
    PTH=`dirname $PTH`
    mkdir -p $CTRL
    cp $PTH/../controllers/$PCTRL/$PCTRL.* $CTRL/puppy
    cp $PTH/../controllers/$SCTRL/$SCTRL.* $CTRL/supervisor
    
    # get the filename extension of the scripts
    #filename=$(basename "$fullfile")
    #extension="${filename##*.}"
    #filename="${filename%.*}"
fi
