#!/bin/bash

# This is an example script how you can send submissions to the robot in a
# somewhat automated way.  The basic idea is the following:
#
#  1. submit job to robot
#  2. wait until job is finished
#  3. download relevant data from the run
#  4. run some algorithm on the data (e.g. to render a video)
#  6. goto 1 unless the desired number of jobs has been submitted
#
# To avoid overloading our system, you MUST NOT poll the server at a higher rate
# than once per minute to see the status of the job!

# expects the following arguments:
# - output directory (to save downloaded files)
# - number of jobs to submit
# - path to "video_creation_image.sif" (for creating videos)
if (( $# != 3 ))
then
    echo "Invalid number of arguments."
    echo "Usage:  $0 <output_directory> <num_submissions> <path_to_apptainer_image>"
    exit 1
fi


output_directory="$1"
num_submissions="$2"


if ! [ -d "${output_directory}" ]
then
    echo "${output_directory} does not exist or is not a directory"
    exit 1
fi

# prompt for username and password (to avoid having user credentials in the
# bash history)
read -p "Username: " username
read -sp "Password: " password
# there is no automatic new line after the password prompt
echo

# you can also put your username and password here
#username="username"
#password="password"


# URL to the webserver at which the recorded data can be downloaded
base_url=https://robots.real-robot-challenge.com/output/${username}/data


# Check if the file/path given as argument exists in the "data" directory of
# the user
function curl_check_if_exists()
{
    local filename=$1

    http_code=$(curl -sI --user ${username}:${password} -o /dev/null -w "%{http_code}" ${base_url}/${filename})

    case "$http_code" in
        200|301) return 0;;
        *) return 1;;
    esac
}


# Send submissions in a loop.
for (( i=0; i<$((num_submissions)); i++))
do
    echo "Submit job"
    submit_result=$(ssh -T ${username}@robots.real-robot-challenge.com <<<submit)
    job_id=$(echo ${submit_result} | grep -oP 'job\(s\) submitted to cluster \K[0-9]+')
    if [ $? -ne 0 ]
    then
        echo "Failed to submit job.  Output:"
        echo "${submit_result}"
        exit 1
    fi
    echo "Submitted job with ID ${job_id}"

    echo "Wait for job to be started"
    job_started=0
    # wait for the job to start (abort if it did not start after half an hour)
    for (( j=0; j<30; j++))
    do
        # Do not poll more often than once per minute!
        sleep 60

        # wait for the job-specific output directory
        if curl_check_if_exists ${job_id}
        then
            job_started=1
            break
        fi
        date
    done

    if (( ${job_started} == 0 ))
    then
        echo "Job did not start."
        exit 1
    fi

    echo "Job is running.  Wait until it is finished"
    # if the job did not finish 10 minutes after it started, something is
    # wrong, abort in this case
    job_finished=0
    for (( j=0; j<15; j++))
    do
        # Do not poll more often than once per minute!
        sleep 60

        # report.json is explicitly generated last of all files, so once this
        # exists, the job has finished
        if curl_check_if_exists ${job_id}/report.json
        then
            job_finished=1
            break
        fi
        date
    done

    # create directory for this job
    job_dir="${output_directory}/${job_id}"
    mkdir "${job_dir}"
    mkdir "${job_dir}/user"

    if (( ${job_finished} == 0 ))
    then
        echo "Job did not finish in time."
        exit 1
    fi

    echo "Job ${job_id} finished."

    echo "Download data to ${job_dir}"

    # Download data. 
    for file in report.json info.json goal.json robot_data.dat camera_data.dat camera60.yml camera180.yml camera300.yml user_stderr.txt user_stdout.txt
    do
        curl --user ${username}:${password} -o "${job_dir}/${file}" ${base_url}/${job_id}/${file}
    done
    curl --user ${username}:${password} -o "${job_dir}/user/goals.json" ${base_url}/${job_id}/user/goals.json

    # if there was a problem with the backend, download its output and exit
    if grep -q "true" "${job_dir}/report.json"
    then
        echo "ERROR: Backend failed!  Download backend output and stop"
        curl --user ${username}:${password} -o "${job_dir}/stdout.txt" ${base_url}/../stdout.txt
        curl --user ${username}:${password} -o "${job_dir}/stderr.txt" ${base_url}/../stderr.txt

        # exit 1
    fi

    # make video (can add additional data processing here)
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    apptainer run "$3" python3 ${SCRIPT_DIR}/trifinger_platform_log_viewer_V2.py ${job_dir}/robot_data.dat ${job_dir}/camera_data.dat -g ${job_dir}/user/goals.json --camera camera60 --save-video ${job_dir}/video.avi

    sleep 60

    echo
    echo "============================================================"
    echo

done