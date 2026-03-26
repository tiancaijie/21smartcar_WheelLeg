代码推送:

cd 路径  （或者直接在工程文件下空处右键git bash here）

git add .

git commit -m "做出了什么修改"

git pull

git push    （git push --force-with-lease origin main  强制推送尽量不要用）


历史文件回退：

git reset --hard HEAD~1（上个版本之前的样子）

或

git reset --hard 你要恢复的版本号

git reflog   所有历史操作记录
