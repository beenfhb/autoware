#ifndef BAGVIEWER_H
#define BAGVIEWER_H

#include <vector>
#include <map>
#include <memory>

#include <QMainWindow>

#include <rosbag/bag.h>

#include "RandomAccessBag.h"


namespace Ui {
class BagViewer;
}

class BagViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit BagViewer(QWidget *parent = 0);
    ~BagViewer();

    void setBagFile(const std::string &bagfilename);

private:
    Ui::BagViewer *ui;

protected:
    std::shared_ptr<rosbag::Bag> bagFdPtr = nullptr;
    RandomAccessBag::Ptr currentActiveTopic = nullptr;
    std::vector<RandomAccessBag::Ptr> imageBagList;
};

#endif // BAGVIEWER_H
