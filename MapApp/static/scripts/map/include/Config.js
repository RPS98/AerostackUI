/**
 * Config class that read all config files and store them in a single object.
 */
class Config 
{
    /**
     * Create a new Config object.
     * @param {string} filePath - Path to the config file to load.
     * @param {function} callback - The callback function to be called when all config files are loaded.
     */
    constructor(filePath, callback) {
        /**
         * The base path of all config files.
         * @type {string}
         * @private
         */
        this._basePath = '/static/scripts/map/Config/';
        
        /**
         * Counter of config files loaded.
         * @type {number}
         * @private
         */
        this._cont = 0;

        /**
         * Number of config files to be loaded.
         * @type {number}
         * @private
         */
        this._dataLenght = null;

        // Load main config file
        Utils.loadLocalFile(this._basePath + filePath, this._intialize.bind(this), 'json', callback);
    }

    /**
     * For each config file in the main file, load it and store it in the Config object.
     * @param {JSON} data - JSON object with all config files, been the key the name of the member in the Config object and the value the path to the config file.
     * @param {function} callback - The callback function to be called when all config files are loaded.
     * @private
     * @return {void}
     */
    _intialize(data, callback) {
        this._dataLenght = Object.keys(data).length;

        for (let i = 0; i < Object.keys(data).length; i++) {
            let key = Object.keys(data)[i];
            let filePath = data[key];
            Utils.loadLocalFile(this._basePath + filePath, this._onLoadConfigFile.bind(this), 'json', key, callback[0]);
        }
    }

    /**
     * Store the config file in the Config object.
     * @param {JSON} data - JSON object with the config file content.
     * @param {list} args - List with the callback function to be called when all config files are loaded.
     */
    _onLoadConfigFile(data, args) {
        this[args[0]] = data;

        this._cont++;
        if (this._cont == this._dataLenght) {
            args[1]();
        }
    }
}
