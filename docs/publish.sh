!/bin/bash
git push

ghp-import -n -p -f _build/html
echo "Website pushed to gh-pages branch"