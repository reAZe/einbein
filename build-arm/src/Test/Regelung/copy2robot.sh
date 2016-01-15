#!/bin/bash
. lib.sh.in

setc remote_user "root"
setc remote_host "es139.ntb.ch"
setc remote_dir "einbein/03_Test_Regelung"

scp Trajektorie/Trajektorie $remote_user@$remote_host:$remote_dir
scp Base2Tool/Fusspunkt $remote_user@$remote_host:$remote_dir
scp Controller/Controller $remote_user@$remote_host:$remote_dir
scp PDV/PDV $remote_user@$remote_host:$remote_dir
scp Encoder/Encoder $remote_user@$remote_host:$remote_dir 
scp Controll $remote_user@$remote_host:$remote_dir 
scp IMU/IMU $remote_user@$remote_host:$remote_dir
scp ZustBest/Zustand $remote_user@$remote_host:$remote_dir
