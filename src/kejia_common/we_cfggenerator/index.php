<?php

//include smarty libraries
require('./libs/Smarty.class.php');

//include user defined functions
foreach(glob("./functions/*.php") as $filename){
	include_once $filename;
}

//get the tplname
$tplname = $argv[1];

$smarty = new Smarty;
$smarty->clearAllCache();
$smarty->left_delimiter="{*";
$smarty->right_delimiter="*}";

$arr_key = array();
//include variables
foreach(glob("./variables/*.php") as $filename){
    $params = include $filename;
    foreach($params as $key=>$value){

        foreach($arr_key as $k=>$fn){
            if($k == $key){
                error_log( "$key has defined in $fn before. Please choose another name in $filename.");
                die();
            }
        }
        $arr_key[$key]=$filename;

        $smarty->assign($key,$value);
    }
}

$smarty->display($tplname);
