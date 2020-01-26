#include "options.h"
#include "ui_options.h"

#define LL_DEFAULT_LIVE_SUFFIX "_live"
#define GUACA_DEFAULT_PORT 40204
#define MOLE_DIR_DEFAULT "/data/mole"
#define RAW_DIR_DEFAULT "/data/rawdir"

extern char configdir[128];

#define new_key "options/customized_2019"
#define old_key "options/customized"

Options::Options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Options),
    settings("guacamole", "guaca")
{
    ui->setupUi(this);
    main = parent;

    // remove old key
    if (settings.contains(old_key)) {
        qDebug() << "Deleting old options";
        settings.remove(old_key);
    }

    // check new key
    if (settings.contains(new_key)) {
        qDebug() << "Restoring saved options";
        restore_options();
    } else {
        qDebug() << "Using default options";
        default_options();
    }

    ui->client_port->setValidator(new QIntValidator(0, 65535, this) );
    ui->server_port->setValidator(new QIntValidator(0, 65535, this) );

    on_buttonBox_clicked(ui->buttonBox->button(QDialogButtonBox::Save));
    if (auto_live) ui->live_name->setEnabled(true);
    else ui->live_name->setDisabled(true);

    completer = new QCompleter(this);
    fsmodel = new QFileSystemModel(completer);
    fsmodel->setRootPath("");
    completer->setModel(fsmodel);
    completer->setCompletionMode(QCompleter::PopupCompletion);
    ui->mole_dir->setCompleter(completer);
    ui->raw_dir->setCompleter(completer);

    restoreGeometry(settings.value("options").toByteArray());
}

Options::~Options() {
    // save the geometry of the options window
    settings.setValue("options", saveGeometry());

    for (unsigned int i=0; i<helpers.size(); i++) {
        if (helpers[i]) delete helpers[i];
        helpers[i] = NULL;
    }
    qDebug() << "Destroying options";
    if (ui) delete ui;
    ui = NULL;
    if (fsmodel) delete fsmodel;
    fsmodel = NULL;
    if (completer) delete completer;
    completer = NULL;
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
    // remove the last entry to signal end of list
    QString prefix = "options/row" + QString::number(i) + "/";
    settings.remove(prefix + "cmdname");
    settings.remove(prefix + "args");
    settings.remove(prefix + "terminal");

    // save mole options
    settings.setValue("options/server", server);
    settings.setValue("options/client", client);
    settings.setValue("options/backup", backup);
    settings.setValue("options/no_checksums", no_checksums);
    settings.setValue("options/mole_terminal", mole_terminal);

    settings.setValue("options/server_port", server_port);
    settings.setValue("options/client_port", client_port);
    settings.setValue("options/live_name", live_name);
    settings.setValue("options/auto_live", auto_live);
    settings.setValue("options/mole_dir", mole_dir);
    settings.setValue("options/raw_dir", raw_dir);

    settings.setValue(new_key, true);
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
    ui->server->setChecked(QVariant(settings.value("options/server", QVariant(false))).toBool());
    ui->client->setChecked(QVariant(settings.value("options/client", QVariant(true))).toBool());
    ui->backup->setChecked(QVariant(settings.value("options/backup", QVariant(false))).toBool());
    ui->checksum->setChecked(QVariant(settings.value("options/no_checksums", QVariant(false))).toBool());
    ui->mole_terminal->setChecked(QVariant(settings.value("options/mole_terminal", QVariant(false))).toBool());

    ui->server_port->setText(QString::number(QVariant(settings.value("options/server_port", QVariant(GUACA_DEFAULT_PORT))).toInt()));
    ui->client_port->setText(QString::number(QVariant(settings.value("options/client_port", QVariant(GUACA_DEFAULT_PORT))).toInt()));
    ui->live_name->setText(QVariant(settings.value("options/live_name", QVariant(MOLE_DIR_DEFAULT))).toString());
    ui->auto_live->setChecked(QVariant(settings.value("options/auto_live", QVariant(true))).toBool());
    ui->mole_dir->setText(QVariant(settings.value("options/mole_dir", QVariant(MOLE_DIR_DEFAULT))).toString());
    ui->raw_dir->setText(QVariant(settings.value("options/raw_dir", QVariant(RAW_DIR_DEFAULT))).toString());
}

void Options::default_options()
{
    // purge all leftover helpers
    ui->tableWidget->setRowCount(0);

    add_helper("its", "-B", false, ui->tableWidget->rowCount());
    add_helper("katie", "-b -m -c mcc_log", true, ui->tableWidget->rowCount());
    add_helper("katie", "-b -m -c ifc_log", true, ui->tableWidget->rowCount());

    // default mole options
    ui->server->setChecked(false);
    ui->client->setChecked(true);
    ui->backup->setChecked(false);
    ui->checksum->setChecked(false);
    ui->mole_terminal->setChecked(false);

    ui->server_port->setText(QString::number(GUACA_DEFAULT_PORT));
    ui->client_port->setText(QString::number(GUACA_DEFAULT_PORT));
    ui->live_name->setText(QString(LL_DEFAULT_LIVE_SUFFIX));
    ui->auto_live->setChecked(true);
    ui->mole_dir->setText(MOLE_DIR_DEFAULT);
    ui->raw_dir->setText(RAW_DIR_DEFAULT);
}

void Options::load_options()
{
    // purge all leftover helpers
    ui->tableWidget->setRowCount(0);

    // add all helpers
    for (unsigned int i=0; i<helpers.size(); i++) {
        add_helper(helpers[i]->cmdname, helpers[i]->args, helpers[i]->terminal, ui->tableWidget->rowCount());
    }

    // sets mole options
    ui->server->setChecked(server);
    ui->client->setChecked(client);
    ui->backup->setChecked(backup);
    ui->checksum->setChecked(no_checksums);
    ui->mole_terminal->setChecked(mole_terminal);

    ui->server_port->setText(QString::number(server_port));
    ui->client_port->setText(QString::number(client_port));
    ui->live_name->setText(live_name);
    ui->auto_live->setChecked(auto_live);
    ui->mole_dir->setText(mole_dir);
    ui->raw_dir->setText(raw_dir);
}

void Options::apply_options() {
    // apply new helpers
    int num_rows = ui->tableWidget->rowCount();
    for (int i=0; i<num_rows; i++) {
        Helper *h = new Helper(ui->tableWidget, i);
        helpers.push_back(h);
        // qDebug() << helpers[i].cmdname + " " + helpers[i].args;
    }

    // apply new options
    server = ui->server->isChecked();
    client = ui->client->isChecked();
    backup = ui->backup->isChecked();
    no_checksums = ui->checksum->isChecked();
    mole_terminal = ui->mole_terminal->isChecked();

    // apply port (0 is not allowed)
    server_port = ui->server_port->text().toInt();
    server_port = (!server_port) ? 40204 : server_port;
    client_port = ui->client_port->text().toInt();
    client_port = (!client_port) ? 40204 : client_port;

    live_name = ui->live_name->text();
    auto_live = ui->auto_live->isChecked();
    mole_dir = ui->mole_dir->text();
    mole_dir = (mole_dir.isEmpty()) ? MOLE_DIR_DEFAULT : mole_dir;
    raw_dir = ui->raw_dir->text();
    raw_dir = (raw_dir.isEmpty()) ? RAW_DIR_DEFAULT : raw_dir;
}

void Options::enable_options() {
    ui->checksum->setEnabled(true);
    ui->server->setEnabled(true);
    ui->client->setEnabled(true);
    ui->backup->setEnabled(true);
    ui->mole_terminal->setEnabled(true);

    ui->client_port->setEnabled(true);
    ui->server_port->setEnabled(true);

    ui->auto_live->setEnabled(true);
    if (auto_live) ui->live_name->setEnabled(true);
    ui->mole_dir->setEnabled(true);
    ui->raw_dir->setEnabled(true);
}

void Options::disable_options() {
    ui->checksum->setDisabled(true);
    ui->server->setDisabled(true);
    ui->client->setDisabled(true);
    ui->backup->setDisabled(true);
    ui->mole_terminal->setDisabled(true);

    ui->client_port->setDisabled(true);
    ui->server_port->setDisabled(true);

    ui->auto_live->setDisabled(true);
    ui->live_name->setDisabled(true);
    ui->mole_dir->setDisabled(true);
    ui->raw_dir->setDisabled(true);
}

void Options::start_helpers() {
    for (unsigned int i=0; i<helpers.size(); i++) {
        // close any straggling logscrolls
        if (helpers[i]->log) delete helpers[i]->log;
        helpers[i]->log = NULL;
        start_a_helper(helpers[i]);
    }
}

void Options::start_a_helper(Helper * helper) {
    // open new logscroll
    QString helper_cmd = helper->cmdname + " "+
                         helper->args;
    helper->log = new Logscroll(NULL, helper_cmd);
    helper->log->setWindowTitle(helper_cmd);
    if (helper->terminal) {
        helper->log->show();
    }
}

void Options::show_helpers() {
    for (unsigned int i=0; i<helpers.size(); i++) {
        if (helpers[i]->log->doneProcess()) {
            start_a_helper(helpers[i]);
        }
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
    log = NULL;
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
            if (helpers[i]) delete helpers[i];
            helpers[i] = NULL;
        }
        helpers.clear();
        if (button == ui->buttonBox->button(QDialogButtonBox::Save)) {
            apply_options();
        } else if (button == ui->buttonBox->button(QDialogButtonBox::RestoreDefaults)) {
            default_options();
        }
        save_options();
        start_helpers();
    }
}

void Options::on_auto_live_toggled(bool checked)
{
    auto_live = checked;
    if (checked) ui->live_name->setEnabled(true);
    else ui->live_name->setDisabled(true);
}
