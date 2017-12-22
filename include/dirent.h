#ifndef __DIRENT_H
#define __DIRENT_H

struct dirent {
	char d_name[256];
};

typedef struct dir {
	struct device_d *dev;
	struct fs_driver_d *fsdrv;
	struct node_d *node;
	struct dirent d;
	void *priv; /* private data for the fs driver */
} DIR;

DIR *opendir(const char *pathname);
struct dirent *readdir(DIR *dir);
int closedir(DIR *dir);

#endif /* __DIRENT_H */
