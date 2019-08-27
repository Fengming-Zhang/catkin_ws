1.简介
	smarty是一个php模板渲染引擎，本工具借用smarty完成对配置文件的渲染。
2.文件结构
	functions:	自定义函数目录，可用于对数据做预处理。
	libs:		smarty library目录。
	templates:	配置文件模板目录。
	variables:	全局变量文件目录。
	templates_c:smarty临时目录。
	index.php:	程序入口。
	make.sh：	运行脚本。
3.使用方法
	a.准备配置文件模板。将配置文件(例如we_laser.yaml)放入templates文件夹。
	b.准备变量。在variables文件夹中新建php文件，文件需要返回一个php数组，数组的key为变量名，value为变量的值。也可在已有php文件中添加所需变量。
	c.测试。确保系统已安装php，在终端中运行php index.php we_laser.yaml，会输出渲染之后的模板，可在此时检查是否符合需求。
	d.运行配置。在basedir中配置KeJia2文件夹所在目录。在make.sh的templates数组中添加模板文件对应目标配置文件的路径。
	e.运行。在终端中运行./make.sh we_laser.yaml，会根据上一步配置的路径将生成的配置文件写入目标位置。也可运行./make.sh all更新make.sh中配置的所有文件。
4.补充说明
	a.php代码以<?php开始，以?>结束。数组格式为array(key=>value),key建议使用字符串，value可为字符串，也可为数组。
	b.在模板中使用变量，需在变量名前加$，并用{**}包围变量。例如，在variables中定义了"topic_name"=>"scan"，在模板中使用{*$topic_name*}即可得到scan。
	c.在模板中使用逻辑语句。最实用的是foreach，可用于快速生成相似代码段。语法格式如下：
		{*foreach $allnames as $sex=>$names*}
			{*foreach $names as $name*}
			<lexis concept="{*$sex*}" pos="NN" predgen="{*$name*}($$).">
				<literal>{*$name*}</literal>
			</lexis>
			{*/foreach*}
		{*/foreach*}
	上面的一段代码将根据variables配置的allnames变量生成<lexis concept..... </lexis>代码段。
	更多的语法格式可以参考http://www.smarty.net/docs/zh_CN/。本工具为了和python文件兼容，将smarty默认的分隔符{}修改为{**}，在文档中对应替换即可。