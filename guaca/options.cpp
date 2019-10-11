#include "options.h"
#include "ui_options.h"

extern char configdir[128];

Options::Options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Options),
    settings("SuperBIT", "guaca")
{
    ui->setupUi(this);
    main = parent;

    if (settings.contains("options/customized")) {
        qDebug() << "Restoring saved options";
        restore_options();
    } else {
        qDebug() << "Using default options";
        default_options();
    }

    on_buttonBox_clicked(ui->buttonBox->button(QDialogButtonBox::Save));
}

Options::~Options() {
    for (unsigned int i=0; i<helpers.size(); i++) {
        delete helpers[i];
    }
    qDebug() << "Destroying options";
    delete ui;
}

unsigned int Options::add_helper(QString cmdname, QString args, bool terminal, unsigned int row) {
    ui->tableWidget->insertRow(row);
    ui->tableWidget->setItem(row, 0, new QTableWidgetItem(cmdname));
    ui->tableWidget->setItem(row, 1, new QTableWidgetItem(args));

    QWidget *checkBoxWidget = new QWidget();
    QHBoxLayout *layoutCheckBox = new QHBoxLayout(checkBoxWidget); // create a layer with reference to the widget
    QCheckBox * cb = new QCheckBox();
    layoutCheckBox->addWidget(cb);            // Set the checkbox in the layer
    layoutCheckBox->setAlignment(Qt::AlignCenter);  // Center the checkbox
    layoutCheckBox->setContentsMargins(0,0,0,0);    // Set the zero padding
    cb->setChecked(terminal);

    // Set the checkbox in the second column
    ui->tableWidget->setCellWidget(row,2,cb);

    return row;
}

unsigned int Options::remove_helper(unsigned int row) {
    ui->tableWidget->removeRow(row);
    return row;
}

void Options::save_options()
{
    unsigned int i = 0;
    for (i=0; i<helpers.size(); i++) {
        QString prefix = "options/row" + QString::number(i) + "/";
        settings.setValue(prefix + "cmdname", helpers[i]->cmdname);
        settings.setValue(prefix + "args", helpers[i]->args);
        settings.setValue(prefix + "terminal", helpers[i]->terminal);
    }
    QString prefix = "options/row" + QString::number(i) + "/";
    settings.remove(prefix + "cmdname");
    settings.remove(prefix + "args");
    settings.remove(prefix + "terminal");

    settings.setValue("options/customized", true);
}

void Options::restore_options()
{
    unsigned int i = 0;
    QString prefix = "options/row" + QString::number(i) + "/";
    while (settings.contains(prefix + "cmdname")) {
        QString cmdname = QVariant(settings.value(prefix + "cmdname")).toString();
        QString args =    QVariant(settings.value(prefix + "args")).toString();
        bool terminal =   QVariant(settings.value(prefix + "terminal")).toBool();
        add_helper(cmdname, args, terminal, ui->tableWidget->rowCount());
        prefix = "options/row" + QString::number(++i) + "/";
    }
}

void Options::default_options()
{
    // purge all leftover helpers
    ui->tableWidget->setRowCount(0);

    add_helper("its", "-B", false, ui->tableWidget->rowCount());
    add_helper("katie", "-b -m -c mcc_log", true, ui->tableWidget->rowCount());
    add_helper("katie", "-b -m -c ifc_log", true, ui->tableWidget->rowCount());
}

void Options::load_options()
{
    // purge all leftover helpers
    ui->tableWidget->setRowCount(0);

    // add all helpers
    for (unsigned int i=0; i<helpers.size(); i++) {
        add_helper(helpers[i]->cmdname, helpers[i]->args, helpers[i]->terminal, ui->tableWidget->rowCount());
    }

}

void Options::start_helpers() {
    for (unsigned int i=0; i<helpers.size(); i++) {
        // start the command process
        QString helper_cmd = helpers[i]->cmdname + " "+
                             helpers[i]->args;

        qDebug() << helper_cmd;

        // close any straggling logscrolls
        if (helpers[i]->log) delete helpers[i]->log;

        // open new logscroll
        helpers[i]->log = new Logscroll(NULL, helper_cmd);
        helpers[i]->log->setWindowTitle(helper_cmd);
        if (helpers[i]->terminal) {
            helpers[i]->log->show();
        }
    }
}

void Options::show_helpers() {
    for (unsigned int i=0; i<helpers.size(); i++) {
        if (helpers[i]->terminal) {
            helpers[i]->log->show();
        }
    }
}

void Options::hide_helpers() {
    for (unsigned int i=0; i<helpers.size(); i++) {
        helpers[i]->log->hide();
    }
}

Helper::Helper(QTableWidget *tw, int index) {
    cmdname = tw->item(index,0)->text();
    args = tw->item(index,1)->text();
    QCheckBox *cb = qobject_cast<QCheckBox*>(tw->cellWidget(index,2));
    terminal = cb->isChecked();
    this->log = NULL;
}

Helper::Helper(QString cmdname, QString args, bool terminal) {
    this->cmdname = cmdname;
    this->args = args;
    this->terminal = terminal;
    this->log = NULL;
}

Helper::~Helper() {
    qDebug() << "Destroying helper";
    if (log) delete log;
}


void Options::on_addHelper_clicked()
{
    add_helper("", "", false, ui->tableWidget->currentRow()+1);
}

void Options::on_deleteHelper_clicked()
{
    remove_helper(ui->tableWidget->currentRow());
}

void Options::on_buttonBox_clicked(QAbstractButton *button)
{
    if (button != ui->buttonBox->button(QDialogButtonBox::Close)) {
        // clear old helpers
        for (unsigned int i=0; i<helpers.size(); i++) {
            delete helpers[i];
        }
        helpers.clear();
        if (button == ui->buttonBox->button(QDialogButtonBox::Save)) {
            // load new helpers
            int num_rows = ui->tableWidget->rowCount();
            for (int i=0; i<num_rows; i++) {
                Helper *h = new Helper(ui->tableWidget, i);
                helpers.push_back(h);
                // qDebug() << helpers[i].cmdname + " " + helpers[i].args;
            }
        } else if (button == ui->buttonBox->button(QDialogButtonBox::RestoreDefaults)) {
            default_options();
        }
        save_options();
        start_helpers();
    }
}
