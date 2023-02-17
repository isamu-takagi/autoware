#!/bin/bash
set -ex
git pull
vcs import src <autoware.repos
vcs pull src
echo "Completed."
