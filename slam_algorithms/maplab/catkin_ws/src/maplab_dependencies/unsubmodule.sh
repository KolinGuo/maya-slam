var=$(git config --file .gitmodules --get-regexp path | awk '{ print $2 }')

IFS=$'\n' read -rd '' -a a <<<"$var"
# now loop through the above array
for i in "${a[@]}"
do
    echo "$i"
    # echo "$i"/.git
    git rm -r --cached "$i"/
    rm -rf "$i"/.git
    git add "$i"/
done
