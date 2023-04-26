#!/usr/bin/python3

import os
import re
import shutil
import subprocess
import sys
import json

project_name = 'dv-ros'
build_dir = 'build'
source_dir = 'source'


def call_cmd(cmd):
    result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = result.stdout.decode('utf-8').strip()
    print('\n======\nCalled: ' + cmd + '\nWith output:\n' + output + '\n======\n')
    return output


def sphinx_config_valid(ref):
    result = call_cmd('git ls-tree %s %s/conf.py' % (ref, source_dir))
    if 'conf.py' in result:
        return True
    else:
        return False


def build_docs(ref):
    dest_dir = build_dir + os.path.sep + ref

    if os.path.isdir(dest_dir):
        return

    print('building: ' + ref)
    call_cmd('git checkout --force --recurse-submodules ' + ref)

    if not os.path.isfile(source_dir + os.path.sep + 'conf.py'):
        print('No Sphinx configuration present, skipping: ' + ref)
        return

    # Valid branch, Sphinx configuration exists, so let's build.
    # First run doxygen
    call_cmd('rm -Rf doxy-xml ; doxygen')

    # Update project information
    with open(source_dir + os.path.sep + 'version.py', 'w', encoding='utf-8') as f:
        f.write('project_name = "%s"\n' % project_name)
        f.write('current_version = "%s"\n' % ref)

    # Run sphinx-build for all types
    call_cmd('sphinx-build -M html %s %s' % (source_dir, dest_dir))
    call_cmd('sphinx-build -M epub %s %s' % (source_dir, dest_dir))
    call_cmd('LATEXOPTS="-interaction=batchmode" sphinx-build -M latexpdf %s %s' % (source_dir, dest_dir))

    # Move generated files
    call_cmd('mv %s/epub/%s.epub %s/%s-%s.epub ; rm -Rf %s/epub/' %
             (dest_dir, project_name, dest_dir, project_name, ref, dest_dir))
    call_cmd('mv %s/latex/%s.pdf %s/%s-%s.pdf ; rm -Rf %s/latex/' %
             (dest_dir, project_name, dest_dir, project_name, ref, dest_dir))
    call_cmd('mv %s/html/* %s/ ; rm -Rf %s/html/ ; rm -Rf %s/doctrees/' % (dest_dir, dest_dir, dest_dir, dest_dir))


# get list of all git tags and all remote branches
shell_git_tags = call_cmd('git tag -l')
shell_git_branches = call_cmd('git branch -l -r | grep -v "\\->"')

git_tags = shell_git_tags.split('\n')
git_branches = shell_git_branches.split('\n')

# check for current branch or tag name (first argument)
git_current = None

if len(sys.argv) >= 2:
    git_current = sys.argv[1].strip()
else:
    git_current = call_cmd('git branch --show-current')
    git_current = re.sub(r'^.*origin/', '', git_current)

if not git_current:
    raise RuntimeError('Could not determine current tag/branch name, pass by first argument')

# check for default branch name (second argument)
git_default = None

if len(sys.argv) >= 3:
    git_default = sys.argv[2].strip()
else:
    git_default = call_cmd('git symbolic-ref --short refs/remotes/origin/HEAD')
    git_default = re.sub(r'^.*origin/', '', git_default)

if not git_default:
    raise RuntimeError('Could not determine default branch name, pass by second argument')

# clean up branch names, remove up to last "origin/"
git_branches = [re.sub(r'^.*origin/', '', b) for b in git_branches if 'origin/' in b]

# filter to remove ones that cannot generate documentation due to not having sphinx configuration
git_doc_tags = [tag for tag in git_tags if sphinx_config_valid('tags/' + tag)]
git_doc_branches = [branch for branch in git_branches if sphinx_config_valid('origin/' + branch)]

git_doc_tags.sort()
git_doc_branches.sort()

# debug
print('\n======')
print('All tags:')
print(git_tags)
print('All branches:')
print(git_branches)
print('Current: ' + git_current)
print('Default: ' + git_default)
print('Filtered tags:')
print(git_doc_tags)
print('Filtered branches:')
print(git_doc_branches)
print('======\n')

# if build/ directory exists, we assume it's a cache, so we'll only build missing tags/branches
# the current branch is *always* rebuilt (by forcing the build)
os.makedirs(build_dir, exist_ok=True)

if os.path.isdir(build_dir + os.path.sep + git_current):
    shutil.rmtree(build_dir + os.path.sep + git_current)

existing_docs = [e for e in os.listdir(build_dir) if os.path.isdir(build_dir + os.path.sep + e)]
print('Existing docs:')
print(existing_docs)

all_buildable_docs = set(git_doc_tags).union(git_doc_branches).union([git_current])
all_existing_docs = set(existing_docs)

docs_to_remove = all_existing_docs.difference(all_buildable_docs)
docs_to_build = [git_current]  # Only build current doc branch to save time.
print('Final to remove:')
print(docs_to_remove)
print('Final to build:')
print(docs_to_build)

for ref in docs_to_remove:
    shutil.rmtree(build_dir + os.path.sep + ref)

for ref in docs_to_build:
    build_docs(ref)

# reset to original current branch
call_cmd('git checkout --force --recurse-submodules ' + git_current)

# ensure default branch is always first one, if applicable
if git_default in git_doc_branches:
    git_doc_branches.remove(git_default)
    git_doc_branches.insert(0, git_default)
else:
    git_default = git_current

# write out versions info to JSON file
with open(build_dir + os.path.sep + 'versions.json', 'w', encoding='utf-8') as f:
    json.dump({
        'tags': git_doc_tags,
        'branches': git_doc_branches,
        'default': git_default
    },
              f,
              ensure_ascii=False,
              indent=4)

# update root level redirect
with open(build_dir + os.path.sep + 'index.html', 'w', encoding='utf-8') as f:
    f.write('<!DOCTYPE html>\n')
    f.write('<html>\n')
    f.write('<head>\n')
    f.write('<title>%s documentation redirect</title>\n' % project_name)
    f.write('<meta http-equiv="refresh" content="0; url=\'%s/index.html\'" />\n' % git_default)
    f.write('</head>\n')
    f.write('<body>\n')
    f.write('<p>Please wait while we redirect you to our <a href="%s/index.html">latest documentation</a>.</p>\n' %
            git_default)
    f.write('</body>\n')
    f.write('</html>\n')
