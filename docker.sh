#!/bin/bash

# Fix for rootless Podman on SLURM clusters (no D-Bus session)
if [ -z "$XDG_RUNTIME_DIR" ]; then
    export XDG_RUNTIME_DIR="/tmp/${USER}-runtime-$$"
    mkdir -p "$XDG_RUNTIME_DIR"
fi
export DBUS_SESSION_BUS_ADDRESS=""

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
    local cpus="16"
    local mem="32G"
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

reattach() {
    local jobid=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -j|--jobid)
                jobid="$2"
                shift 2
                ;;
            -h|--help)
                reattach_usage
                exit 0
                ;;
            *)
                # If no flag, assume it's a job ID
                if [[ -z "$jobid" ]]; then
                    jobid="$1"
                    shift
                else
                    print_error "Unknown option: $1"
                    reattach_usage
                    exit 1
                fi
                ;;
        esac
    done

    # If no job ID provided, try to find a running job
    if [[ -z "$jobid" ]]; then
        print_info "No job ID specified, searching for running SLURM jobs..."
        
        # Get list of running jobs for current user
        local running_jobs
        running_jobs=$(squeue -u "$USER" -h -t RUNNING -o "%i %j %N %M" 2>/dev/null)
        
        if [[ -z "$running_jobs" ]]; then
            print_error "No running SLURM jobs found for user $USER"
            print_info "Use './docker.sh salloc' to start a new allocation"
            exit 1
        fi

        local job_count
        job_count=$(echo "$running_jobs" | wc -l)

        if [[ "$job_count" -eq 1 ]]; then
            jobid=$(echo "$running_jobs" | awk '{print $1}')
            local jobname=$(echo "$running_jobs" | awk '{print $2}')
            local node=$(echo "$running_jobs" | awk '{print $3}')
            print_info "Found running job: $jobid ($jobname) on node $node"
        else
            print_info "Multiple running jobs found:"
            echo ""
            printf "  %-12s %-20s %-15s %-10s\n" "JOBID" "NAME" "NODE" "TIME"
            echo "  --------------------------------------------------------"
            echo "$running_jobs" | while read -r line; do
                printf "  %-12s %-20s %-15s %-10s\n" $line
            done
            echo ""
            print_error "Please specify which job to reattach to with: ./docker.sh reattach <JOBID>"
            exit 1
        fi
    fi

    # Verify the job exists and is running
    local job_state
    job_state=$(squeue -j "$jobid" -h -o "%t" 2>/dev/null)
    
    if [[ -z "$job_state" ]]; then
        print_error "Job $jobid not found or no longer exists"
        exit 1
    fi

    if [[ "$job_state" != "R" ]]; then
        print_error "Job $jobid is not running (state: $job_state)"
        exit 1
    fi

    local node
    node=$(squeue -j "$jobid" -h -o "%N" 2>/dev/null)
    
    print_info "Reattaching to job $jobid on node $node..."
    print_info "Starting container shell..."

    # Use srun to run a command on the allocated node
    srun --jobid="$jobid" --pty bash -c "cd ${SCRIPT_DIR} && ${SCRIPT_DIR}/docker.sh init"
}

reattach_usage() {
    echo "Usage: $0 reattach [JOBID] [options]"
    echo ""
    echo "Reattach to an existing SLURM allocation and container."
    echo ""
    echo "Options:"
    echo "  -j, --jobid JOBID    Job ID to reattach to (optional if only one job running)"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 reattach              # Auto-detect single running job"
    echo "  $0 reattach 123456       # Reattach to specific job ID"
    echo "  $0 reattach -j 123456    # Same as above"
}

usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build    - Build the Docker image"
    echo "  init     - Initialize and start the container (with X11 support)"
    echo "  salloc   - Request a SLURM interactive node and start container there"
    echo "  reattach - Reattach to an existing SLURM allocation and container"
    echo "  delete   - Delete the container"
    echo "  stop     - Stop the running container"
    echo "  shell    - Open a shell in the running container"
    echo ""
    echo "SLURM options (for 'salloc' command):"
    echo "  -a, --account ACCOUNT    Account to charge (default: share-ie-idi)"
    echo "  -n, --nodes NODES        Number of nodes (default: 1)"
    echo "  -p, --partition PART     Partition to use (default: CPUQ)"
    echo "  -c, --cpus CPUS          Number of CPUs (default: 4)"
    echo "  -m, --mem MEMORY         Memory allocation (default: 16G)"
    echo "  -t, --time TIME          Time limit (default: 03:00:00)"
    echo ""
    echo "Reattach options:"
    echo "  -j, --jobid JOBID        Job ID to reattach to (auto-detect if omitted)"
    echo ""
    echo "Examples:"
    echo "  ./docker.sh salloc --mem 32G --time 06:00:00 --cpus 8"
    echo "  ./docker.sh reattach           # Auto-detect running job"
    echo "  ./docker.sh reattach 123456    # Reattach to specific job"
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
    reattach)
        shift
        reattach "$@"
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
