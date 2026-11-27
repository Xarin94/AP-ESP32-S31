/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  ArduPilot filesystem interface for esp32 systems
 */
#include "AP_Filesystem.h"
#include <AP_HAL/AP_HAL.h>

#if AP_FILESYSTEM_ESP32_ENABLED

#define FSDEBUG 0

#include <utime.h>
#include <AP_HAL_ESP32/SdCard.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// a format holds fs_sem: the calls made meanwhile fail with EBUSY instead of
// waiting for it, except close and closedir, which wait so that no
// descriptor is lost
#define FS_CHECK_FORMAT(retval) do { if (formatting()) { errno = EBUSY; return retval; } } while (0)

int AP_Filesystem_ESP32::open(const char *fname, int flags, bool allow_absolute_paths)
{
#if FSDEBUG
    printf("DO open %s \n", fname);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    // we automatically add O_CLOEXEC as we always want it for ArduPilot FS usage
    return ::open(fname, flags | O_TRUNC | O_CLOEXEC, 0666);
}

int AP_Filesystem_ESP32::close(int fd)
{
#if FSDEBUG
    printf("DO close \n");
#endif
    WITH_SEMAPHORE(fs_sem);
    return ::close(fd);
}

int32_t AP_Filesystem_ESP32::read(int fd, void *buf, uint32_t count)
{
#if FSDEBUG
    printf("DO read \n");
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::read(fd, buf, count);
}

int32_t AP_Filesystem_ESP32::write(int fd, const void *buf, uint32_t count)
{
#if FSDEBUG
    printf("DO write \n");
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::write(fd, buf, count);
}

int AP_Filesystem_ESP32::fsync(int fd)
{
#if FSDEBUG
    printf("DO fsync \n");
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::fsync(fd);
}

int32_t AP_Filesystem_ESP32::lseek(int fd, int32_t offset, int seek_from)
{
#if FSDEBUG
    printf("DO lseek \n");
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::lseek(fd, offset, seek_from);
}

int AP_Filesystem_ESP32::stat(const char *pathname, struct stat *stbuf)
{
#if FSDEBUG
    printf("DO stat %s \n", pathname);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::stat(pathname, stbuf);
}

int AP_Filesystem_ESP32::unlink(const char *pathname)
{
#if FSDEBUG
    printf("DO unlink %s \n", pathname);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::unlink(pathname);
}

int AP_Filesystem_ESP32::rename(const char *oldpath, const char *newpath)
{
#if FSDEBUG
    printf("DO rename %s \n", oldpath, newpath);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::rename(oldpath, newpath);
}

int AP_Filesystem_ESP32::mkdir(const char *pathname)
{
#if FSDEBUG
    printf("DO mkdir %s \n", pathname);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    return ::mkdir(pathname, 0777);
}

void* AP_Filesystem_ESP32::opendir(const char *pathname)
{
#if FSDEBUG
    printf("DO opendir %s \n", pathname);
#endif
    FS_CHECK_FORMAT(nullptr);
    WITH_SEMAPHORE(fs_sem);

    return (void*)::opendir(pathname);
    //	return NULL;
}

struct dirent *AP_Filesystem_ESP32::readdir(void *dirp)
{
#if FSDEBUG
    printf("DO readdir \n");
#endif
    FS_CHECK_FORMAT(nullptr);
    WITH_SEMAPHORE(fs_sem);
    return ::readdir((DIR*)dirp);
    //	return NULL;
}

int AP_Filesystem_ESP32::closedir(void *dirp)
{
#if FSDEBUG
    printf("DO closedir \n");
#endif
    WITH_SEMAPHORE(fs_sem);

    return ::closedir((DIR*)dirp);
    //	return 0;
}

// return number of bytes that should be written before fsync for optimal
// streaming performance/robustness. if zero, any number can be written.
// every HAL_ESP32_FS_FSYNC_SIZE bytes
uint32_t AP_Filesystem_ESP32::bytes_until_fsync(int fd)
{
    FS_CHECK_FORMAT(0);
    WITH_SEMAPHORE(fs_sem);
    int32_t pos = ::lseek(fd, 0, SEEK_CUR);
    if (pos < 0) {
        return 0;
    }

    const uint32_t block_size = HAL_ESP32_FS_FSYNC_SIZE;

    uint32_t block_pos = (uint32_t)pos % block_size;
    return block_size - block_pos;
}

// return free disk space in bytes
int64_t AP_Filesystem_ESP32::disk_free(const char *path)
{

#if FSDEBUG
    printf("DO free disk %s \n", path);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    FATFS *fs;
    DWORD fre_clust, fre_sect;

    /* Get volume information and free clusters of sdcard */
    auto res = f_getfree("/SDCARD/", &fre_clust, &fs);
    if (res) {
        return -1;
    }

    /* Get total sectors and free sectors */
    fre_sect = fre_clust * fs->csize;

    return (int64_t)fre_sect * FF_SS_SDCARD;
}

// return total disk space in bytes
int64_t AP_Filesystem_ESP32::disk_space(const char *path)
{
#if FSDEBUG
    printf("DO usage disk %s \n", path);
#endif
    FS_CHECK_FORMAT(-1);
    WITH_SEMAPHORE(fs_sem);
    FATFS *fs;
    DWORD fre_clust, tot_sect;

    /* Get volume information and free clusters of sdcard */
    auto res = f_getfree("/SDCARD/", &fre_clust, &fs);
    if (res) {
        return -1;
    }

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;

    return (int64_t)tot_sect * FF_SS_SDCARD;
}

/*
  set mtime on a file
 */
bool AP_Filesystem_ESP32::set_mtime(const char *filename, const uint32_t mtime_sec)
{

#if FSDEBUG
    printf("DO time %s \n", filename);
#endif
    FS_CHECK_FORMAT(false);
    WITH_SEMAPHORE(fs_sem);
    struct utimbuf times {};
    times.actime = mtime_sec;
    times.modtime = mtime_sec;

    return utime(filename, &times) == 0;
}

/*
  format the SD card
 */
bool AP_Filesystem_ESP32::format(void)
{
    if (formatting()) {
        return false;
    }
    // the format is handled asynchronously in the IO thread, we inform the
    // user of success via a text message. format_status can be polled for
    // progress
    format_status = FormatStatus::PENDING;
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Filesystem_ESP32::format_handler, void));
    return true;
}

void AP_Filesystem_ESP32::format_handler(void)
{
    if (format_status != FormatStatus::PENDING) {
        return;
    }
    // wait for the calls in progress, the new ones fail
    WITH_SEMAPHORE(fs_sem);
    format_status = FormatStatus::IN_PROGRESS;
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Formatting SDCard");
    if (format_sdcard()) {
        format_status = FormatStatus::SUCCESS;
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Format: OK");
    } else {
        format_status = FormatStatus::FAILURE;
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Format: Failed");
    }
}

AP_Filesystem_Backend::FormatStatus AP_Filesystem_ESP32::get_format_status(void) const
{
    return format_status;
}

bool AP_Filesystem_ESP32::formatting(void) const
{
    return format_status == FormatStatus::PENDING ||
           format_status == FormatStatus::IN_PROGRESS;
}

#endif  // AP_FILESYSTEM_ESP32_ENABLED
