#!/bin/sh
POSIXLY_CORRECT=1

BASEDIR=`echo ${0} | sed 's|/\?[^/][^/]*[[:blank:]]*$||g'`

# drop leading slash
DIR="`echo ${1} | sed 's|//*$||g'`"
DAYS="${2}"

if [ -z "${DIR}" ] ||
    [ -z "${DAYS}" ]
then
    echo "Usage: ${0} <directory> <number-of-days>"
    exit 1
fi


# script stuff

HOURS="`echo "${DAYS} * 24" | bc`"
MINUTES="`echo "${HOURS} * 60" | bc`"
SECONDS="`echo "${MINUTES} * 60" | bc`"

list_if_old() {
    ls -ldA "${1}" | grep "repo" 2>&1 > /dev/null || :
    IS_REPO="${?}"
    TIMESTAMP=`ls -ldA --time-style=+%s "${1}" | \
        awk '{print $6}' | \
        grep -v total`
    DATE=`date +%s`
    if [ -n "${DATE}" ] && \
       [ "${TIMESTAMP}" -lt "$(( ${DATE} - ${SECONDS} ))" ]
    then
        echo "${1}"
    fi
}

list_old_files() {
    for i in `ls -A ${1}`
    do
        list_if_old "${1}/${i}"
    done
}

find_old_files() {
    for i in `ls -A ${1}`
    do
        if [ -d "${1}/${i}" ] && [ ! -h "${1}/${i}" ]
        then
            find_old_files "${1}/${i}"
        else
            list_if_old "${1}/${i}"
        fi
    done
}

find_old_files "${DIR}"

