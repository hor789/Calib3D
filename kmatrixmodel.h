#ifndef KMATRIXMODEL_H
#define KMATRIXMODEL_H

#include <QObject>
#include <QAbstractListModel>
#include "kmatrix.h"

class KMatrixModel : public QAbstractListModel
{
    Q_OBJECT

public:
    KMatrixModel(QObject *parent = 0, KMatrix *core = 0);

    KMatrix *core();
    void clear();
    int rowCount(const QModelIndex &) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;

private slots:
    void onCoreDataChanged();

private:
    KMatrix *coreData;
};

#endif // KMATRIXMODEL_H
