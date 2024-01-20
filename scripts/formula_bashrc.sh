#!/usr/bin/env bash

export FORMULA_HOME=$(
	cd "$(dirname "${BASH_SOURCE[0]}")/.."
	pwd -P
)

# unlimited bash history size
export HISTFILESIZE=
export HISTSIZE=
