#!/bin/bash

# Replace with the actual URL of the raw file
curl -o ../../src/prolog_reasoning/know_plan_reasoning/plan_comparison.pl https://github.com/albertoOA/know_plan/blob/main/src/prolog_reasoning/comparative/plan_comparison.pl
curl -o ../../src/prolog_reasoning/know_plan_reasoning/__init__.pl https://github.com/albertoOA/know_plan/blob/main/src/prolog_reasoning/comparative/__init__.pl

echo "File updated!"
