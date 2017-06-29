#!/bin/bash
# we need to start from share dir, because server tries to load some relative paths
cd ~/osvr/share/osvrcore 
../../bin/osvr_server sample-configs/osvr_server_config.UnifiedVideoTracker.HDK2UpgradeKitDirect.json

