#!/usr/bin/env bash

# Requires having installed flypush w/ its setup script first, from:
# https://github.com/tom-f-oconnell/flypush

sudo -u flypush psql -f setup.sql
