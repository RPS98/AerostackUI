// List class of [type, fileName]
var blocksClassList = []

class HTMLUtils {

    static setDict(type, id = 'none', attributes = {}, content) {
        return {
            'type': type,
            'id': id,
            'attributes': attributes,
            'content': content,
        }
    }

    static addTypeDict(type, id = 'none', attributes = {}, ...args) {
        throw new Error('Not implemented');
    }

    static addTypeHTML(content) {
        throw new Error('Not implemented');
    }

    static _addAttribute(element, id, attributes) {
        if (id != 'none') {
            element.setAttribute('id', id);
        }
        for (let key in attributes) {
            if (element.getAttribute(key) == null) {
                element.setAttribute(key, attributes[key]);
            } else {
                element.setAttribute(key, element.getAttribute(key) + ' ' + attributes[key]);
            }
        }
        return element;
    }

    static addDict(type, id = 'none', attributes = {}, ...args) {
        for (let i = 0; i < blocksClassList.length; i++) {
            if (blocksClassList[i][0] == type) {
                return blocksClassList[i][1].addTypeDict(type, id, attributes, ...args);
            }
        }
    }

    static addHTML(parent, childDict) {
        let child = null;
        let flag = true;

        if (childDict == undefined) {
            return;
        }

        if (Object.keys(childDict).length === 0) {
            return;
        }

        if (Array.isArray(childDict)) {
            for (let j = 0; j < childDict.length; j++) {
                HTMLUtils.addHTML(parent, childDict[j])
            }
            return;
        }

        for (let i = 0; i < blocksClassList.length; i++) {
            if (blocksClassList[i][0] == childDict.type) {
                child = blocksClassList[i][1].addTypeHTML(childDict.content);
                HTMLUtils._addAttribute(child, childDict.id, childDict.attributes);
                parent.appendChild(child);
                flag = false;
                break; // If types are unique, break after first match to optimize performance
            }
        }

        if (flag) {
            console.log("Unknown type of HTML block");
            console.log(childDict)
            console.log(typeof childDict)
            console.log(Array.isArray(childDict))
            throw new Error('Unknown type of HTML block');
        }
    }

    static dictToHTML(dict) {
        let child = null;

        for (let i = 0; i < blocksClassList.length; i++) {
            if (blocksClassList[i][0] == dict.type) {
                child = blocksClassList[i][1].addTypeHTML(dict.content);
                HTMLUtils._addAttribute(child, dict.id, dict.attributes);
                return child;
            }
        }

        console.log("Unknown type of HTML block");
        console.log(dict)
        throw new Error('Unknown type of HTML block');
    }

    static addToExistingElement(id, blockList) {
        let parent = document.getElementById(id);

        for (let i = 0; i < blockList.length; i++) {
            HTMLUtils.addHTML(parent, blockList[i]);
        }
    }

    static initDropDown(id, list, defaultValue) {
        let dropDownBtn = HTMLUtils.addDict('dropDownBtn', `${id}-DropDown-Btn`, { 'class': 'btn btn-info' }, defaultValue);

        let dropDownExpandList = [];
        for (let i = 0; i < list.length; i++) {
            dropDownExpandList.push(HTMLUtils.addDict('button', `${id}-DropDown-Expand-${list[i]}`, { 'class': `btn btn-secondary ${id}-item w-75`, 'style': "background: #dae8fc" }, list[i]));
        }

        let dropDownExpand = HTMLUtils.addDict('dropDownExpand', `${id}-DropDown-Expand`, {}, dropDownExpandList);
        return HTMLUtils.addDict('dropDown', `${id}-DropDown`, { 'class': 'row m-1 gap-2' }, dropDownBtn, dropDownExpand);
    }

    static updateDropDown(id, list) {
        let expand = document.getElementById(`${id}-DropDown-menu`);
        expand.innerHTML = '';

        let dropDownExpandList = [];
        for (let i = 0; i < list.length; i++) {
            dropDownExpandList.push(HTMLUtils.addDict('button', `${id}-DropDown-Expand-${list[i]}`, { 'class': `btn btn-secondary ${id}-item w-75`, 'style': "background: #dae8fc" }, list[i]));
        }

        let dropDownExpand = HTMLUtils.addDict('dropDownExpand', `${id}-DropDown-Expand`, {}, dropDownExpandList);
        HTMLUtils.addToExistingElement(`${id}-DropDown-menu`, [dropDownExpand]);
    }

    static updateCheckBoxes(idCheckBoxes, type, list, attributes = {}) {
        console.log("updateCheckBoxes")
        console.log(list)
        let checkBoxes = document.getElementById(`${idCheckBoxes}`);
        checkBoxes.innerHTML = '';
        for (let i = 0; i < list.length; i++) {
            let checkBox = HTMLUtils.addDict('checkBox', `${idCheckBoxes}-${list[i]}`, attributes, type, list[i]);
            HTMLUtils.addToExistingElement(`${idCheckBoxes}`, [checkBox]);
        }
    }

    static addCheckBox(idParent, idContent, type, name, attributes = {}) {
        let parentElement = document.getElementById(idParent);
        HTMLUtils.addHTML(parentElement, HTMLUtils.addDict('checkBox', `${idContent}-${name}`, attributes, type, name));
    }
}


class Utils {

    /**
     * Call each function in the list with the arguments
     * @param {array} callbackList - list of pair with the function to be called and the arguments to be passed
     * @param {...any} args - other arguments to be passed to the functions
     * @return {void} 
     * @access public
     * @static
     */
    static callCallbacks(callbackList, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            callbackList[i][0](callbackList[i][1], args);
        }
    }

    /**
     * Call each function in the list if the param changed is the desired one.
     * If desired param is '*' this function will be also called.
     * @param {array} callbackList - list of functions to be called 
     * @param {string} param - name of the param that changed
     * @param {any} value - value of the param that changed
     * @param {...any} args - other arguments to be passed to the functions
     * @return {void} 
     * @access public
     * @static
     */
    static callCallbackByParam(callbackList, param, value, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            if (callbackList[i][0] == param || callbackList[i][0] == '*') {
                callbackList[i][1](param, value, args);
            }
        }
    }

    /**
     * Dumb function to initialize callbacks
     * @param {Object} input - functions inputs 
     */
    static _nullFunct(input) {
        console.log("Dumb callback function");
    };

    /**
     * Add a callback function to a toggle button
     * @param {string}   button_id  - id of the button
     * @param {function} callback   - callback when press button
     * @param {list}     args       - arguments list pass to callback
     */
    static addButtonCallback(button_id, callback = Utils._nullFunct, ...args) {
        // get button element
        const btn = document.getElementById(button_id);

        // add an event listener to the button
        if (btn != null) {
            btn.addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the button callback
                callback(args);
            });
        } else {
            console.log("Warning: Utils.addButtonCallback - button not found");
        }
    }

    static addButtonsCallback(button_class, callback = Utils._nullFunct, ...args) {
        const btns = document.getElementsByClassName(button_class);

        // add an event listener to the button
        for (let i = 0; i < btns.length; i++) {
            btns[i].addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the button callback
                callback(e.target, args);
            });
        }
    }

    /**
     * Add a listener for number inputs by a button
     * @param {string}   button_id - id of the button
     * @param {string}   inputs_id - list of form inputs id
     * @param {string}   names_id  - name of elements in the list
     * @param {function} callback  - callback when press button
     * @param {list}     args      - arguments list pass to callback
     */
    static addFormCallback(button_id, inputs_id, names_id, callback = Utils._nullFunct, ...args) {

        // get button element
        const btn = document.getElementById(button_id);

        // check if inputs and names has the same length
        if (inputs_id.length != names_id.length) {
            throw "Input id and names id lengths are not equals";
        }

        if (btn != null) {
            // add a button listener
            btn.addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // for each input elemenet, get the value
                let inputs = [];
                for (let i = 0; i < inputs_id.length; i++) {
                    inputs[names_id[i]] = parseFloat(document.getElementById(inputs_id[i]).value);
                }

                // call the callback
                callback(args, inputs);
            });
        } else {
            console.log("Warning: Utils.addFormCallback - button not found");
        }
    }

    static addFileCallback(button_id, callback = Utils._nullFunct, ...args) {
        const btn = document.getElementById(button_id);

        if (btn != null) {
            btn.addEventListener('change', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the button callback
                callback(args, e.target.files[0]);
            });
        } else {
            console.log("Warning: Utils.addFileCallback - button not found");
        }
    }

    static addlatLngCallback(button_id, input_id, callback = Utils._nullFunct, ...args) {
        // get button element
        const btn = document.getElementById(button_id);

        // add a button listener
        if (btn != null) {
            btn.addEventListener('click', (e) => {
                // disable the refresh on the page when submit
                e.preventDefault();

                // call the callback
                callback(args, document.getElementById(input_id).value);
            });
        }
    }

    static setColor(map, UAV_color) {
        map.pm.setPathOptions({ color: UAV_color });
    }

    static onlyUnique(value, index, self) {
        return self.indexOf(value) === index;
    }

    static readFileText(path) {
        let output = null;
        fetch(path)
            .then(response => response.text())
            .then(data => {
                output = data;
                //console.log(output)
            });
        return output;
    }

    static readFileImage(path) {
        let output = null;
        fetch(path)
            .then(response => response.blob())
            .then(data => {
                output = data;
            });
        return output;
    }

    static readFileJson(path) {
        let output = null;
        fetch(path)
            .then(response => response.json())
            .then(data => {
                output = data;
            });
        return output;
    }

    static round(value, decimals) {
        return Number(Math.round((value + Number.EPSILON) + 'e' + decimals) + 'e-' + decimals);
    }

    static distance(x1, y1, x2, y2) {

        // Euclidean distance
        let x = x2 - x1;
        let y = y2 - y1;
        return Math.sqrt(x * x + y * y);
    }
}


/**
 * Class that own a list of id and a dictionary of values for each id
 */
class SmartList {
    /**
     * Create a new instance of the class SmartList
     */
    constructor() {
        /**
         * List of id
         * @type {array}
         * @access private
         */
        this._objectList = [];

        /**
         * Dictionary of values for each id (key: id, value: value)
         * @type {dict}
         * @access private
         */
        this._objectDict = {};
    }

    /**
     * Add an id and its value
     * @param {string} id - Id to be added
     * @param {dict} object - Value to be added
     * @returns {void}
     * @access public
     */
    addObject(id, object) {
        this._objectList.push(id);
        this._objectDict[id] = object;
    }

    /**
     * Change the value of an id
     * @param {string} id - Id to be changed
     * @param {dict} objectInfo - Value to be added
     * @returns {void}
     * @access public
     */
    updateObject(id, objectInfo) {
        this._objectDict[id] = Object.assign({}, this._objectDict[id], objectInfo);
    }

    /**
     * Return the list of id
     * @returns {array} - List of id
     * @access public
     */
    getList() {
        return this._objectList;
    }

    /**
     * Return the dictionary of values for each id
     * @returns {dict} - Dictionary of values for each id
     * @access public
     */
    getDict() {
        return this._objectDict;
    }

    /**
     * Get the value of an id
     * @param {string} id - Id to be added
     * @returns {void}
     * @access public
     */
    getDictById(id) {
        if (this.getList().indexOf(id) !== -1) {
            return this._objectDict[id];
        } else {
            return null;
        }
    }

    /**
     * Remove an id and its value
     * @param {string} id - Id to be removed
     * @returns {void}
     * @access public
     */
    removeObject(id) {
        let index = this._objectList.indexOf(id);
        if (index > -1) {
            this._objectList.splice(index, 1);
        }
        delete this._objectDict[id];
    }
}


/**
 * UAV and Mission Manager prototype, that manage income information from server and call desired callbacks.
 */
class SmartListCallbacks extends SmartList {

    /**
     * Create a new instance of the class ManagerPrototype.
     * @param {string} infoAdd - Name of the header of the info message that will be received from server when a parameter is add/modified.
     * @param {string} infoSet - Name of the header of the info message that will be received from server when the information is set/reset.
     * @param {string} infoGet - Name of the header of the request message that will be received from server when the information is requested.
     **/
    constructor() {

        super();

        /**
         * List of callbacks when a parameter is modified
         * @type {array}
         * @private
         */
        this._infoChangeCallbacks = [];

        /**
         * List of callbacks when a object is added or removed
         * @type {array}
         * @private
         */
        this._infoAddCallbacks = [];

        /**
         * List of callbacks when the a parameter is changed
         * @type {array}
         * @private
         */
        this._paramChangeCallbacks = [];
    }

    // #region Public methods

    /**
     * Add a function callback when any parameter is changed.
     * @param {function} callback - Function to be called when the parameter is changed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoChangeCallback(callback, ...args) {
        this._infoChangeCallbacks.push([callback, args]);
    }

    /**
     * Add a function callback when new information is added.
     * @param {function} callback - Function to be called when the information is added.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoAddCallback(callback, ...args) {
        this._infoAddCallbacks.push([callback, args]);
    }

    /**
     * Add a function callback when the desired parameter is changed.
     * @param {string} param - Parameter to be watched.
     * @param {function} callback - Function to be called when the parameter is changed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoParamCallback(param, callback, ...args) {
        this._paramChangeCallbacks.push([param, callback, args]);
    }

    /**
     * Remove the information with the given id.
     * @param {any} id - Id of the information.
     * @return {void}
     * @access public 
     */
    removeById(id) {
        // console.log("Utils - removeById");
        // console.log(id);
        Utils.callCallbacks(this._infoAddCallbacks, id, 'remove');
        super.removeObject(id);
    }

    /**
     * Add an id and its value
     * @param {string} id - Id to be added
     * @param {dict} object - Value to be added
     * @returns {void}
     * @access public
     */
    addObject(id, objectInfo) {
        super.addObject(id, objectInfo);
        Utils.callCallbacks(this._infoAddCallbacks, id, 'add');

        for (let key in objectInfo) {
            Utils.callCallbackByParam(this._paramChangeCallbacks, key, objectInfo[key], id);
        }
    }

    /**
     * Change the value of an id
     * @param {string} id - Id to be changed
     * @param {dict} objectInfo - Value to be added
     * @returns {void}
     * @access public
     */
    updateObject(id, objectInfo, override = false) {

        if (override) {
            super.addObject(id, objectInfo);
            Utils.callCallbacks(this._infoChangeCallbacks, id);
        } else {
            super.updateObject(id, objectInfo);
            Utils.callCallbacks(this._infoChangeCallbacks, id);
        }

        let oldInfo = this.getDictById(id);
        let newInfo = objectInfo;

        for (let key in objectInfo) {
            if (!(key in oldInfo)) {
                if (oldInfo[key] !== newInfo[key]) {
                    Utils.callCallbackByParam(this._paramChangeCallbacks, key, objectInfo[key], id);
                }
            }
        }

        // for (let key in objectInfo) {
        //     Utils.callCallbackByParam(this._paramChangeCallbacks, key, objectInfo[key], id);
        // }
    }
    // #endregion
}
