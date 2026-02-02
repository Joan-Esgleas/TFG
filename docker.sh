#!/bin/bash


IMAGE_NAME="silicon-tfg"
CONTAINER_NAME="silicon-tfg-container"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

build() {
    print_info "Building Docker image: ${IMAGE_NAME}..."
    docker build -t "${IMAGE_NAME}" "${SCRIPT_DIR}"
    if [ $? -eq 0 ]; then
        print_info "Image built successfully!"
    else
        print_error "Failed to build image"
        exit 1
    fi
}

init() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            print_info "Container is already running. Attaching..."
            docker exec -it "${CONTAINER_NAME}" /bin/bash
            return
        else
            print_info "Starting existing container..."
            docker start "${CONTAINER_NAME}"
            docker exec -it "${CONTAINER_NAME}" /bin/bash
            return
        fi
    fi

    print_info "Creating and starting new container: ${CONTAINER_NAME}..."

    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth

    if [ ! -f "${XAUTH}" ]; then
        touch "${XAUTH}"
        xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge - 2>/dev/null
    fi

    if [ -z "$DISPLAY" ]; then
        print_warn "DISPLAY variable not set. X11 forwarding may not work."
        print_warn "Make sure to connect with 'ssh -X' or 'ssh -Y' to the cluster."
    fi

    docker run -it \
        --name "${CONTAINER_NAME}" \
        --hostname "${CONTAINER_NAME}" \
        -e DISPLAY="${DISPLAY}" \
        -e XAUTHORITY="${XAUTH}" \
        -v "${XSOCK}:${XSOCK}:rw" \
        -v "${XAUTH}:${XAUTH}:rw" \
        -v "${SCRIPT_DIR}:/workspace:rw" \
        -v "${HOME}/.Xauthority:/root/.Xauthority:rw" \
        --net=host \
        --ipc=host \
        "${IMAGE_NAME}" \
        /bin/bash

    if [ $? -ne 0 ]; then
        print_error "Failed to start container"
        exit 1
    fi
}

delete() {
    print_info "Deleting container: ${CONTAINER_NAME}..."
    
    # Stop container if running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_info "Stopping running container..."
        docker stop "${CONTAINER_NAME}"
    fi

    # Remove container
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker rm "${CONTAINER_NAME}"
        print_info "Container deleted successfully!"
    else
        print_warn "Container does not exist"
    fi
}

stop() {
    print_info "Stopping container: ${CONTAINER_NAME}..."
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker stop "${CONTAINER_NAME}"
        print_info "Container stopped successfully!"
    else
        print_warn "Container is not running"
    fi
}

shell() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_info "Opening shell in running container..."
        docker exec -it "${CONTAINER_NAME}" /bin/bash
    else
        print_error "Container is not running. Use './docker.sh init' first."
        exit 1
    fi
}

salloc_init() {
    local account="share-ie-idi"
    local nodes="1"
    local partition="CPUQ"
    local cpus="4"
    local mem="16G"
    local time="03:00:00"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -a|--account)
                account="$2"
                shift 2
                ;;
            -n|--nodes)
                nodes="$2"
                shift 2
                ;;
            -p|--partition)
                partition="$2"
                shift 2
                ;;
            -c|--cpus)
                cpus="$2"
                shift 2
                ;;
            -m|--mem)
                mem="$2"
                shift 2
                ;;
            -t|--time)
                time="$2"
                shift 2
                ;;
            -h|--help)
                salloc_usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                salloc_usage
                exit 1
                ;;
        esac
    done

    print_info "Requesting interactive SLURM node..."
    print_info "  Account:   ${account}"
    print_info "  Partition: ${partition}"
    print_info "  Nodes:     ${nodes}"
    print_info "  CPUs:      ${cpus}"
    print_info "  Memory:    ${mem}"
    print_info "  Time:      ${time}"
    echo ""

    # Run salloc and then start docker inside the allocated node
    salloc --account="${account}" \
           --nodes="${nodes}" \
           --partition="${partition}" \
           -c "${cpus}" \
           --mem="${mem}" \
           --time="${time}" \
           bash -c "cd ${SCRIPT_DIR} && ${SCRIPT_DIR}/docker.sh init"
}

salloc_usage() {
    echo "Usage: $0 salloc [options]"
    echo ""
    echo "Options:"
    echo "  -a, --account ACCOUNT    Account to charge (default: share-ie-idi)"
    echo "  -n, --nodes NODES        Number of nodes (default: 1)"
    echo "  -p, --partition PART     Partition to use (default: CPUQ)"
    echo "  -c, --cpus CPUS          Number of CPUs (default: 4)"
    echo "  -m, --mem MEMORY         Memory allocation (default: 16G)"
    echo "  -t, --time TIME          Time limit (default: 03:00:00)"
    echo "  -h, --help               Show this help message"
    echo ""
    echo "Example:"
    echo "  $0 salloc --mem 32G --time 06:00:00 --cpus 8"
}

usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build   - Build the Docker image"
    echo "  init    - Initialize and start the container (with X11 support)"
    echo "  salloc  - Request a SLURM interactive node and start container there"
    echo "  delete  - Delete the container"
    echo "  stop    - Stop the running container"
    echo "  shell   - Open a shell in the running container"
    echo ""
    echo "SLURM options (for 'salloc' command):"
    echo "  -a, --account ACCOUNT    Account to charge (default: share-ie-idi)"
    echo "  -n, --nodes NODES        Number of nodes (default: 1)"
    echo "  -p, --partition PART     Partition to use (default: CPUQ)"
    echo "  -c, --cpus CPUS          Number of CPUs (default: 4)"
    echo "  -m, --mem MEMORY         Memory allocation (default: 16G)"
    echo "  -t, --time TIME          Time limit (default: 03:00:00)"
    echo ""
    echo "Example:"
    echo "  ./docker.sh salloc --mem 32G --time 06:00:00 --cpus 8"
    echo ""
    echo "For X11 forwarding on a cluster, make sure to:"
    echo "  1. Connect with 'ssh -X user@cluster' or 'ssh -Y user@cluster'"
    echo "  2. Have xauth installed on the cluster"
    echo ""
}

# Main script logic
case "$1" in
    build)
        build
        ;;
    init)
        init
        ;;
    salloc)
        shift
        salloc_init "$@"
        ;;
    delete)
        delete
        ;;
    stop)
        stop
        ;;
    shell)
        shell
        ;;
    *)
        usage
        exit 1
        ;;
esac
