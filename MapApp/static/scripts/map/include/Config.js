class Config 
{
    constructor(filePath, callback) {
        this._basePath = '/static/scripts/map/Config/';
        this._cont = 0;
        this._dataLenght = null;
        Utils.loadLocalFile(this._basePath + filePath, this.intialize.bind(this), 'json', callback);
    }

    intialize(data, callback) {
        this._dataLenght = Object.keys(data).length;

        for (let i = 0; i < Object.keys(data).length; i++) {
            let key = Object.keys(data)[i];
            let filePath = data[key];
            Utils.loadLocalFile(this._basePath + filePath, this._onLoadConfigFile.bind(this), 'json', key, callback[0]);
        }
    }

    _onLoadConfigFile(data, args) {
        this[args[0]] = data;

        this._cont++;
        if (this._cont == this._dataLenght) {
            args[1]();
        }
    }
}
