#!/bin/bash
set -e
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../" && pwd )"
TMP_DIR="/tmp"

install_ipopt() {
    echo "Prepare to install IPOPT ..."
    IPOPT_URL="https://e.coding.net/tmp-code/ipopt-3.12.4.git"
    sudo apt-get -y install \
        gfortran \
        cmake  \
        build-essential \
        gcc \
        g++
    sudo ldconfig
    if ( ldconfig -p | grep libipopt ); then
        echo "Ipopt is already installed......."
    else
        echo "Start installing Ipopt, version: 3.12.4  .........."
        pwd
        cd $TMP_DIR
        pwd
        rm -rf ipopt-3.12.4 && git clone "$IPOPT_URL" && cd ipopt-3.12.4
        # configure,build and install the IPOPT
        echo "Configuring and building IPOPT ..."
        ./configure --prefix /usr/local
        make -j$(nproc)
        make test
        sudo make install
        if (grep 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' $HOME/.bashrc); then
          echo "LD_LIBRARY_PATH has been set."
        else
          echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> $HOME/.bashrc
        fi
        sudo ldconfig
        echo "IPOPT installed successfully"
        source $HOME/.bashrc
    fi
    cd $REPO_DIR
}

main() {
    #sudo apt-get update
    install_ipopt
    # install_cppad
    # install_benchmark
    # install_glog
    # install_gflags
    # install_grid_map
    # install_osqp_eigen
    # clone_other_ros_pkgs
}

main