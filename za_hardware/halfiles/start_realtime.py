#!/usr/bin/env python3

from machinekit import hal

# Make sure the realtime runtime is started only after whole configuration
# is loaded and not sooner

hal.start_threads()
