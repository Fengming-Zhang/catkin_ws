#!/bin/bash

declare -A templates

#templates[conf.py]='we_brainstem/we_python_sm/src/conf.py'
#templates[we_laser.yaml]='we_hardware/we_laser/cfg/we_laser.yaml'
#templates[motor_manager.yaml]='we_hardware/we_motor_manager/cfg/motor_manager.yaml'
#templates[gslam.yaml]='we_navigation/we_gslam/cfg/gslam.yaml'
templates[noun.xml]=`rospack find we_nlu`/res/lexicon/noun.xml
templates[name]=`rospack find we_nlu`/res/name
templates[nluobject]=`rospack find we_nlu`/res/taxonomy/object.xml
templates[scene_b.xml]=`rospack find we_worldmodel`/res/scene_b.xml
templates[object]=`rospack find we_taskplan`/res/conf/object
templates[bigobj]=`rospack find we_taskplan`/res/conf/bigobj
templates[smallobj]=`rospack find we_taskplan`/res/conf/smallobj
templates[objclass]=`rospack find we_taskplan`/res/conf/objclass
templates[human]=`rospack find we_taskplan`/res/conf/human
templates[askname]=`rospack find we_speechcfg`/gram/askname.bnf
templates[genpurpose]=`rospack find we_speechcfg`/gram/genpurpose.bnf
templates[order]=`rospack find we_speechcfg`/gram/order.bnf
#templates[wake]=`rospack find we_speechcfg`/gram/wake.bnf
#templates[party]=`rospack find we_speechcfg`/gram/party.bnf
templates[autosave]=`rospack find we_python_sm`/src/locations/autosave.py
#templates[shopping]=`rospack find we_homespeech`/gram/shopping.bnf
for name in $*;do
    if [ $name = "all" ]
    then
        for i in "${!templates[@]}"; do
            echo "generate $i to ${templates[$i]}"
            php index.php ${templates[$i]}.tpl  > ${templates[$i]}
        done
        break
    fi
    echo "generate $name to ${templates[$name]}"
    php index.php ${templates[$name]}.tpl  > ${templates[$name]}
done
