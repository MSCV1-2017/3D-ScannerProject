#ifndef TDK_EDIT_H
#define TDK_EDIT_H

#include <QObject>
#include <QString>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TDK_Edit : public QObject
{
    Q_OBJECT
public:
    explicit TDK_Edit(QObject *parent = 0);

protected:
    QString     mv_EditId;
    QString     mv_EditDescription;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >    *mv_PointcloudVector;

signals:

public slots:
};

#endif // TDK_EDIT_H
