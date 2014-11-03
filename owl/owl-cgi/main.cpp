#include <iostream>
#include <QSharedMemory>
#include <QSystemSemaphore>
#include <QFile>
#include <QByteArray>
#include <QStringList>
#include <argp.h>
#include <time.h>
#include <unistd.h>

using namespace std;

int fail() {
    cout<<"Content-type: text/plain\n\n";
    cout<<"owl-cgi : retrives data from a running owl session.\n";
    cout<<"For use as a CGI script with argument 'key:object' where object is one of:\n";
    cout<<"'html', 'css', 'data', 'layout'\n";
    return 1;
}

int main(int argc, char** argv) {

    if(argc!=2) return fail();

    QStringList args=QString(argv[1]).split(':');
    if(args.size()!=2) return fail();

    QSystemSemaphore sema("_owlSema"+args[0],1);
    sema.acquire(); //!!

    QSharedMemory* smem=0;
    if(args[1]=="html") {
        cout<<"Content-type: text/html\n\n";
        smem=new QSharedMemory("_owlHtml"+args[0]);
    } else if(args[1]=="css") {
        cout<<"Content-type: text/plain\n\n";
        smem=new QSharedMemory("_owlCSS"+args[0]);
    } else if(args[1]=="data") {
        cout<<"Content-type: application/json\n\n";
        smem=new QSharedMemory("_owlData"+args[0]);
    } else if(args[1]=="layout") {
        cout<<"Content-type: text/xml\n\n";
        smem=new QSharedMemory("_owlLayout"+args[0]);
    }

    int i=0;
    if(!smem||!smem->attach()) {
        sema.release();
        if(++i==100) {
            return fail();
        }
        usleep(1000);
    }

    smem->lock();
    cout<<(char*)smem->data();
    smem->unlock();
    smem->detach();
    delete smem;

    sema.release();

    return 0;
}
