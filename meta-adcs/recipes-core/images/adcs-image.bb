SUMMARY = "BeagleBone Black image for the ADCS testbed"
LICENSE = "MIT"

require recipes-core/images/core-image-minimal.bb

# ssh in to deploy and watch logs; adcs-fsw starts at boot.
# The last three replace the old debug-tweaks: root login with no password.
# Development only; never ship an image like this.
IMAGE_FEATURES += "ssh-server-dropbear allow-empty-password empty-root-password allow-root-login"
IMAGE_INSTALL:append = " adcs-fsw"
