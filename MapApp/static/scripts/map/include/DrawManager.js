// var idCodeDraw = 0;
// var idUserDraw = 0;

class DrawManager {
    constructor(status, type, name, parameters = [], options = {}, layerOptions = {}) {
        this.parameters = parameters
        for (let i = 0; i < parameters.length; i++) {
            let parameter = parameters[i];
            let type = parameter[0];
            let name = parameter[1];
            let value = parameter[2];
            if (options.name !== undefined) {
                value = options.name;
            } else {
                options.name = value;
            }
        }

        this.type = type
        this.codeLayerDrawn = null;

        let optionsStructure = {
            'drawManager': {
                'options': {},
                'instance': {},
            }
        }

        this.options = Object.assign(optionsStructure, layerOptions);
        this.options.drawManager.options = options
        this.options.instance = this;
    }

    mergeOptions(options, layerOptions) {

        let drawOption = Object.assign({}, this.options, layerOptions);
        drawOption.drawManager.options = Object.assign({}, drawOption.drawManager.options, options);
        drawOption.drawManager.instance = this;

        return drawOption;
    }

    codeDraw(values, options = {}, layerOptions = {}) {
        let drawOption = this.mergeOptions(options, layerOptions);

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

    userDraw(options = {}, layerOptions = {}) {
        
        let drawOption = this.mergeOptions(options, layerOptions);

        console.log("DrawManager.userDraw");
        console.log(drawOption)

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

    getHtmlParameters(id, options, parameters) {
        let html = [];
        for (let i = 0; i < parameters.length; i++) {
            let parameter = parameters[i];
            let type = parameter[0];
            let name = parameter[1];
            let value = parameter[2];
            if (options.name !== undefined) {
                value = options.name;
            } else {
                options.name = value;
            }
            let text = parameter[3];

            let param = HTMLUtils.addDict('input', `${id}-${name}`, { 'class': 'form-control', 'required': 'required', 'value': value}, type, name);
            let paramBtn = HTMLUtils.addDict('button', `${id}-${name}Btn`, { 'class': 'btn btn-primary' }, `${text}`);
            let paramDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [param]);
            let paramBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [paramBtn]);
            html.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [paramDiv, paramBtnDiv]));
        }
        return html;
    }

    addParametersCallback(id, parameters, info) {
        for (let i = 0; i < parameters.length; i++) {
            let parameter = parameters[i];
            let type = parameter[0];
            let name = parameter[1];
            let value = parameter[2];
            Utils.addFormCallback( `${id}-${name}Btn`, [`${id}-${name}`], [name], this.parametersCallback.bind(this), name, info);
        }
    }

    parametersCallback(myargs, inputs) {
        let name = myargs[0];
        let info = myargs[1];
        info.drawManager.options[name] = inputs[name];
    }

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
        this.drawInfoInitialize(id, info);
    }

    drawInfoRemove(id) {
        let drawInfoHtml = document.getElementById(`${id}-Collapse`);
        if (drawInfoHtml != null) {
            drawInfoHtml.remove();
        } else {
            console.log("Warning: DrawManager.removeDrawInfo - DrawInfoHtml not found");
        }
    }

    addParametersHtml(id, info) {
        let paramList = this.getHtmlParameters(id, info.drawManager.options, info.drawManager.instance.parameters);
        return HTMLUtils.addDict('collapse', `${id}-ParametersCollapse`, {}, 'Parameters', false, paramList)
    }

    drawInfoGetHtml(id, info, initialHtml = [], endHtml = [], uavPickerType = 'none') {        

        initialHtml.push(this._drawInfoGetValues(id));
        initialHtml.push(this._drawInfoGetHeight(id, info));

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

    drawInfoInitialize(id, info) {
        this._addChangeCallback(id, info);
        this._addRemoveCallback(id, info);
        this._addHeightRangeCallback(id, info);

        // Initialize uav picker
        M.uavPickerInitiliazeCallback(`${id}-UAVPicker`);
        let input = document.getElementById(`${id}-UAVPicker-auto-Input`);
        if (input != null) {
            input.setAttribute('checked', true);
            info.drawManager.options.uavList = {'auto': true};
        }
    }

    // Get Draw Info Html
    _drawInfoGetValues(id) {
        let change = HTMLUtils.addDict('button', `${id}-change`, { 'class': 'btn btn-primary' }, 'Change');
        let remove = HTMLUtils.addDict('button', `${id}-remove`, { 'class': 'btn btn-danger' }, 'Remove');
        let changeDiv = HTMLUtils.addDict('div', `none`, {}, [change]);
        let removeDiv = HTMLUtils.addDict('div', `none`, {}, [remove]);
        return HTMLUtils.addDict('div', `none`, { 'class': 'btn-group d-flex justify-content-evenly', 'role': 'group' }, [changeDiv, removeDiv]);
    }

    _drawInfoGetHeight(id, info) {
        let heightMin = info.drawManager.options.height[0];
        let heightMax = info.drawManager.options.height[1];

        // Height range HTML
        let heightInputMin = HTMLUtils.addDict('input', `${id}-heightInputMin`, { 'class': 'form-control', 'required': 'required', 'value': heightMin }, 'text', heightMin);
        let heightInputMax = HTMLUtils.addDict('input', `${id}-heightInputMax`, { 'class': 'form-control', 'required': 'required', 'value': heightMax }, 'text', heightMax);
        let heightRangeBtn = HTMLUtils.addDict('button', `${id}-heighRangeBtn`, { 'class': 'btn btn-primary' }, 'Set Height (m)');

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
        Utils.addFormCallback(`${id}-heighRangeBtn`, [`${id}-heightInputMin`, `${id}-heightInputMax`], ['heightMin', 'heightMax'], this._updateHeightRangeCallback.bind(this), id, info);
    }

    // Draw Info callbacks  
    _updateHeightRangeCallback(myargs, args) {
        console.log("updateHeightRangeCallback");
        let info = myargs[1];
        console.log(info);
        info.drawManager.options.height = [args.heightMin, args.heightMax];
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