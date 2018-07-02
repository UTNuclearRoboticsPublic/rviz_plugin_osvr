#!/bin/bash
# we need to start from share dir, because server tries to load some relative paths
cd ${OSVR_CORE}/share/osvrcore

# Start OSVR with IR position tracker
../../bin/osvr_server sample-configs/osvr_server_config.UnifiedVideoTracker.HDK2UpgradeKitDirect.json

# Start OSVR without tracking
#../../bin/osvr_server sample-configs/osvr_server_config.renderManager.HDKv2.0.extended.json

