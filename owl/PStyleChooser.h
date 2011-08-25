#ifndef PSTYLECHOOSER_H
#define PSTYLECHOOSER_H

#include <QWidget>
#include <QComboBox>
#include <PStyle.h>

namespace Ui {
    class PStyleChooser;
}

class PStyleChooser : public QWidget
{
    Q_OBJECT
    PStyle** current;
public:
    explicit PStyleChooser(QWidget *parent = 0);
    ~PStyleChooser();
public slots:
    void setNoWidgetStyleRef() { current=0; }
    void setWidgetStyleRef(PStyle*& style);
    void select(QString s);
    void boldLogic(bool);
    void italicLogic(bool);
    void fgLogic();
    void bgLogic();
    void refresh();
private:
    Ui::PStyleChooser *ui;
};

#endif // PSTYLECHOOSER_H
