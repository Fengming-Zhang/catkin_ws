<?php
function getAllNames(){
    $vars = include "variables/noun.php";
    $names = array_merge($vars['allnames']['male'],$vars['allnames']['famale']);
    return $names;
}
