/**
 * Draw Manager, that manage user and code draw of layers, and it info.
 */
class DrawManager {
    /**
     * Create a new DrawManager
     * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
     * @param {string} layerType - Type of layer to draw, for example: 'Marker', 'Line', 'Circle', 'Polygon'.
     * @param {string} layerName - Name of the layer, for example: 'TakeOffPoint', 'LandPoint', 'Path'.
     * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button].
     * @param {dict} options - Options of the Draw Manager.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options.
     */
    constructor(status, layerType, layerName, parameters = [], options = {}, layerOptions = {}) {
        /**
         * List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button].
         * @type {list}
         * @public
         */
        this.parameters = parameters

        // Read parameters and add them to options
        for (let i = 0; i < parameters.length; i++) {
            let parameter = parameters[i];
            // let type = parameter[0];
            let name = parameter[1];
            let value = parameter[2];
            if (options[name] !== undefined) {
                value = options[name];
            } else {
                options[name] = value;
            }
        }

        /**
         * Type of layer to draw, for example: 'Marker', 'Line', 'Circle', 'Polygon'.
         * @type {string}
         * @public
         */
        this.type = layerType;

        /**
         * Instance of the draw layer created.
         * @type {object}
         * @public
         */
        this.codeLayerDrawn = null;

        // Create options structure for the draw manager
        let optionsStructure = {
            'drawManager': {
                'options': {
                    'type': this.type,
                    'name': layerName,
                    'status': status,
                },
                'instance': {},
            }
        }

        /**
         * Options of the Draw Manager. Dict with 'options' and 'instance' keys.
         * @type {dict}
         * @public
         */
        this.options = Object.assign(optionsStructure, layerOptions);

        this.options.drawManager.options = Object.assign(this.options.drawManager.options, options);
        this.options.instance = this;
    }

    /**
     * Create options to be added to the layer by given options and layerOptions. Create a deep copy of them.
     * @param {dict} options - Extra options to add to the Draw Manager. 
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options.
     * @returns {dict} - Options to be pass to layers.
     * @private
     */
    _mergeOptions(options, layerOptions) {
        let drawOption = Object.assign({}, this.options, layerOptions);
        drawOption.drawManager.options = Object.assign({}, drawOption.drawManager.options, options);
        drawOption.drawManager.instance = this;
        return drawOption;
    }

    /**
     * Draw a layer by the given values.
     * @param {L.latlng} values - Leaflet latitude and longitude of the layer (Reference: https://leafletjs.com/).
     * @param {dict} options - Extra options to add to the Draw Manager.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options.
     * @returns {object} - Instance of the layer created.
     * @public
     */
    codeDraw(values, options = {}, layerOptions = {}) {
        let drawOption = this._mergeOptions(options, layerOptions);

        let draw = null;

        switch (this.type) {
            case 'Marker':
                draw = L.marker(values, drawOption);
                break;
            case 'Line':
                draw = L.polyline(values, drawOption);
                break;
            case 'Circle':
                draw = L.circle(values[0], values[1], drawOption);
                break;
            case 'Polygon':
                draw = L.polygon(values, drawOption);
                break;
            default:
                alert("Try to draw from code a type: " + drawOption);
                throw new Error("Unknown type of draw");
        }
        draw.addTo(M.MAP);
        this.codeLayerDrawn = draw;

        if (drawOption.draggable == false) {
            this.codeLayerDrawn.pm.setOptions({ draggable: false });
        }
        return draw;
    }

    /**
     * Enable the draw of the layer by the user in the webpage.
     * @param {dict} options - Extra options to add to the Draw Manager.
     * @param {dict} layerOptions - Options of the layer with Leaflet and PM options.
     * @returns {void}
     * @public
     */
    userDraw(options = {}, layerOptions = {}) {

        let drawOption = this._mergeOptions(options, layerOptions);

        switch (this.type) {
            case 'Marker':
                M.MAP.pm.enableDraw('Marker', drawOption);
                break;
            case 'Line':
                M.MAP.pm.enableDraw('Line', drawOption);
                break;
            case 'Circle':
                M.MAP.pm.enableDraw('Circle', drawOption);
                break;
            case 'Polygon':
                M.MAP.pm.enableDraw('Polygon', drawOption);
                break;
            case 'CircleMarker':
                M.MAP.pm.enableDraw('CircleMarker', drawOption);
                break;
            case 'Rectangle':
                M.MAP.pm.enableDraw('Rectangle', drawOption);
                break;
            default:
                alert("Try to draw from code a type: " + type);
                throw new Error("Unknown type of draw");
        }
    }


    // #region Draw Info

    // #region Parameters management

    /**
     * Generate HTML dict of parameters for the layer.
     * @param {string} id - Base id of the HTML element to add.
     * @param {dict} options - Options of the Draw Manager. Parameters will be added to this options. 
     * @param {list} parameters - List of parameters to add. Each parameter is a list of [type, name, value, text to add in input button].
     * @returns {dict} - HTML dict of parameters for the layer.
     * @private
     */
    _getHtmlParameters(id, options, parameters) {
        let html = [];
        for (let i = 0; i < parameters.length; i++) {
            let parameter = parameters[i];
            let type = parameter[0];
            let name = parameter[1];
            let value = parameter[2];
            let text = parameter[3];

            if (options[name] !== undefined) {
                value = options[name];
            } else {
                options[name] = value;
            }

            if (type == "number") {
                let param = HTMLUtils.addDict('input', `${id}-${name}`, { 'class': 'form-control', 'required': 'required', 'value': value }, type, name);
                let paramBtn = HTMLUtils.addDict('button', `${id}-${name}Btn`, { 'class': 'btn btn-primary' }, `${text}`);
                let paramDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [param]);
                let paramBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [paramBtn]);
                html.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [paramDiv, paramBtnDiv]));
            } else if (type == "list") {
                html.push(HTMLUtils.initDropDown(`${id}-Swarming`, text, value));
            }
        }
        return html;
    }

    /**
     * Add a callback for each paramaeter input.
     * @param {string} id - Base id of the HTML element to add.
     * @param {dict} info - Dict with the layer and the Draw Manager options.
     * @returns {void}
     * @private
     */
    _addParametersCallback(id, info) {
        let parameters = info.drawManager.instance.parameters;
        for (let i = 0; i < parameters.length; i++) {
            let parameter = parameters[i];
            let type = parameter[0];
            if (type == "number") {
                let name = parameter[1];
                Utils.addFormCallback(`${id}-${name}Btn`, [`${id}-${name}`], [name], this._parametersNumberCallback.bind(this), name, info);
            } else if (type == "list") {
                Utils.addButtonsCallback(`${id}-Swarming-item`, this._parametersListCallback.bind(this), id, info);
            }
        }
    }

    /**
     * Callback for the parameters input.
     * @param {list} myargs - List with the name of the parameter and the info dictiornary, with the layer and the Draw Manager.
     * @param {list} inputs - List with a dict with the name of the input and the value.
     * @returns {void}
     * @private
     */
    _parametersNumberCallback(myargs, inputs) {
        let name = myargs[0];
        let info = myargs[1];
        info.drawManager.options[name] = inputs[name];
    }

    /**
     * Callback for the parameter list input.
     * @param {Event} e - Event of the button clicked.
     * @param {list} myargs - List with the base id of the HTML dropdown menu and the info dictionary, with the layer and the Draw Manager.
     * @returns {void}
     * @private
     */
    _parametersListCallback(e, args) {
        let id = args[0];
        let info = args[1];
        info.drawManager.options.algorithm = e.innerText;

        let button = document.getElementById(`${id}-Swarming-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
    }

    // #endregion

    /**
     * Remove the HTML info for this layer.
     * @param {string} id - Base id of the HTML element to remove. 
     * @returns {void}
     * @public
     */
     drawInfoRemove(id) {
        let drawInfoHtml = document.getElementById(`${id}-Collapse`);
        if (drawInfoHtml != null) {
            drawInfoHtml.remove();
        } else {
            console.log("Warning: DrawManager.removeDrawInfo - DrawInfoHtml not found");
        }
    }

    /**
     * Add the HTML info for this layer.
     * @param {string} htmlId - Base id of the HTML element to add.
     * @param {dict} info - Dict with the layer and the Draw Manager options.
     * @param {string} name - Name of the layer.
     * @param {list} initialHtml - List with the HTML to add at the beginning of the info.
     * @param {list} endHtml - List with the HTML to add at the end of the info.
     * @param {string} uavPickerType - Type of the UAV picker, for example 'checkbox' or 'radio'.
     * @returns {void}
     * @public
     */
    drawInfoAdd(htmlId, info, name = info.drawManager.name, initialHtml = undefined, endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.id;
        let drawInfo = this.drawInfoGetHtml(id, info, initialHtml, endHtml, uavPickerType);

        // Check if the draw info is already in the html
        let collapseHtml = document.getElementById(`${id}-Collapse-collapsable`);
        if (collapseHtml != null) {
            collapseHtml.innerHTML = '';
            HTMLUtils.addToExistingElement(`${id}-Collapse-collapsable`, [drawInfo]);
        } else {
            let drawInfoHtml = HTMLUtils.addDict('collapse', `${id}-Collapse`, {}, `${name} ${info.id}`, false, drawInfo);
            HTMLUtils.addToExistingElement(htmlId, [drawInfoHtml]);
        }
        this._drawInfoInitialize(id, info);
    }

    /**
     * Initialize callbacks for the HTML input of the layer.
     * @param {string} id - Base id of the HTML element to add.
     * @param {dict} info - Dict with the layer and the Draw Manager options.
     * @returns {void}
     * @private
     */
    _drawInfoInitialize(id, info) {
        this._addChangeCallback(id, info);
        this._addRemoveCallback(id, info);
        this._addHeightRangeCallback(id, info);
        this._addSpeedCallback(id, info);
        this._addParametersCallback(id, info)

        // Initialize uav picker
        M.uavPickerInitiliazeCallback(`${id}-UAVPicker`);
        let input = document.getElementById(`${id}-UAVPicker-auto-Input`);
        if (input != null) {
            input.setAttribute('checked', true);
            info.drawManager.options.uavList = { 'auto': true };
        }
    }    

    addParametersHtml(id, info) {
        let paramList = this._getHtmlParameters(id, info.drawManager.options, info.drawManager.instance.parameters);
        return HTMLUtils.addDict('collapse', `${id}-ParametersCollapse`, {}, 'Parameters', false, paramList)
    }

    drawInfoGetHtml(id, info, initialHtml = [], endHtml = [], uavPickerType = 'none') {

        initialHtml.push(this._drawInfoGetValues(id));
        initialHtml.push(this._drawInfoGetHeight(id, info));
        initialHtml.push(this._drawInfoGetSpeed(id, info));

        let htmlBody = [];
        htmlBody.push(HTMLUtils.addDict('collapse', `${id}-ValuesCollapse`, {}, 'Modify', false, initialHtml));

        if (this.parameters.length > 0) {
            let html = this.addParametersHtml(id, info);
            if (html != null) {
                htmlBody.push(html);
            }
        }

        htmlBody.push(endHtml);
        if (uavPickerType != 'none') {
            htmlBody.push(this._drawInfoGetUavPicker(id, info, uavPickerType));
        }
        return htmlBody;
    }    

    // Get Draw Info Html
    _drawInfoGetValues(id) {
        let change = HTMLUtils.addDict('button', `${id}-change`, { 'class': 'btn btn-primary' }, 'Change');
        let remove = HTMLUtils.addDict('button', `${id}-remove`, { 'class': 'btn btn-danger' }, 'Remove');
        let changeDiv = HTMLUtils.addDict('div', `none`, {}, [change]);
        let removeDiv = HTMLUtils.addDict('div', `none`, {}, [remove]);
        return HTMLUtils.addDict('div', `none`, { 'class': 'btn-group d-flex justify-content-evenly', 'role': 'group' }, [changeDiv, removeDiv]);
    }

    _drawInfoGetSpeed(id, info) {
        let speed = info.drawManager.options.speed;

        // Seep range HTML
        let speedInput = HTMLUtils.addDict('input', `${id}-speedInput`, { 'class': 'form-control', 'required': 'required', 'value': speed }, 'text', speed);
        let speedBtn = HTMLUtils.addDict('button', `${id}-speedBtn`, { 'class': 'btn btn-primary' }, 'Set speed (m/s)');
        let speedInputDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [speedInput]);
        let speedBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-7' }, [speedBtn]);

        return HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [speedInputDiv, speedBtnDiv]);
    }

    _drawInfoGetHeight(id, info) {
        let heightMin = info.drawManager.options.height[0];
        let heightMax = info.drawManager.options.height[1];

        // Height range HTML
        let heightInputMin = HTMLUtils.addDict('input', `${id}-heightInputMin`, { 'class': 'form-control', 'required': 'required', 'value': heightMin }, 'text', heightMin);
        let heightInputMax = HTMLUtils.addDict('input', `${id}-heightInputMax`, { 'class': 'form-control', 'required': 'required', 'value': heightMax }, 'text', heightMax);
        let heightRangeBtn = HTMLUtils.addDict('button', `${id}-heightRangeBtn`, { 'class': 'btn btn-primary' }, 'Set Height (m)');

        let heightInputMinDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [heightInputMin]);
        let heightInputMaxDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [heightInputMax]);
        let heightRangeBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [heightRangeBtn]);

        return HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [heightInputMinDiv, heightInputMaxDiv, heightRangeBtnDiv]);
    }

    _drawInfoGetUavPicker(id, info, uavPickerType = 'checkbox') {
        let list = [['auto', true]];
        let uavPickerList = M.getUavPickerDict(uavPickerType, `${id}-UAVPicker`, list, this._uavPickerCallback.bind(this), uavPickerType, info);
        return HTMLUtils.addDict('collapse', `${id}-UAVCollapse`, {}, 'UAV Picker', true, [uavPickerList]);
    }


    // Add Draw Info callbacks
    _addChangeCallback(id, info) {
        throw new Error("Not implemented Drawmanager._addChangeCallback");
    }

    _addRemoveCallback(id, info) {
        Utils.addButtonCallback(`${id}-remove`, this._removeCallback.bind(this), info.id);
    }

    _addHeightRangeCallback(id, info) {
        Utils.addFormCallback(`${id}-heightRangeBtn`, [`${id}-heightInputMin`, `${id}-heightInputMax`], ['heightMin', 'heightMax'], this._updateHeightRangeCallback.bind(this), id, info);
    }

    _addSpeedCallback(id, info) {
        Utils.addFormCallback(`${id}-speedBtn`, [`${id}-speedInput`], ['speed'], this._updateSpeedCallback.bind(this), id, info);
    }

    // Draw Info callbacks  
    _updateHeightRangeCallback(myargs, args) {
        console.log("updateHeightRangeCallback");
        let info = myargs[1];
        console.log(info);
        info.drawManager.options.height = [args.heightMin, args.heightMax];
    }

    _updateSpeedCallback(myargs, args) {
        console.log("updateSpeedCallback");
        let info = myargs[1];
        console.log(info);
        info.drawManager.options.speed = args.speed;
    }

    _removeCallback(myargs) {
        console.log("DrawInfo: removeCallback");
        console.log(myargs);
        M.DRAW_LAYERS.removeLayerById(myargs[0]);
    }

    _changeCallback(myargs) {
        throw new Error("Not implemented Drawmanager._changeCallback");
    }

    _uavPickerCallback(uavName, value, userargs) {

        let type = userargs[0];
        let drawManager = userargs[1].drawManager;

        if (type == 'radio') {
            if (value) {
                drawManager.options.uavList = {}
                drawManager.options.uavList[String(uavName)] = true;
            }
        } else {
            drawManager.options.uavList[String(uavName)] = value;
        }
    }

    //#endregion

}