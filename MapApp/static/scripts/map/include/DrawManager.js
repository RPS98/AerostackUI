// var idCodeDraw = 0;
// var idUserDraw = 0;

class DrawManager {
    constructor(type, name, options = {}) {
        this.type = type
        this.options = Object.assign(
            {
                'instance': this,
                'type': type,
                'name': name,
            },
            options,
        );

        this.codeLayerDrawn = null;
    }

    codeDraw(values, options = {}) {
        let drawManagerOptions = Object.assign({}, this.options, { 'drawCodeOptions': options, 'id': 'none' });
        let drawOptions = Object.assign({ 'drawManager': drawManagerOptions }, options);

        let draw = null;

        switch (this.type) {
            case 'Marker':
                draw = L.marker(values, drawOptions);
                break;
            case 'Line':
                draw = L.polyline(values, drawOptions);
                break;
            case 'Circle':
                draw = L.circle(values[0], values[1], drawOptions);
                break;
            case 'Polygon':
                draw = L.polygon(values, drawOptions);
                break;
            default:
                alert("Try to draw from code a type: " + this.type);
                throw new Error("Unknown type of draw");
        }
        draw.addTo(M.MAP);
        this.codeLayerDrawn = draw;
        return draw
    }

    userDraw(options = {}) {
        options['author'] = 'user';
        options['status'] = 'draw';
        options['uavList'] = {};
        //options['continueDrawing'] = true;
        let drawManagerOptions = Object.assign({}, this.options, { 'drawUserOptions': options, 'id': 'none' });
        let drawOptions = Object.assign({ 'drawManager': drawManagerOptions }, options, this.options);

        switch (this.type) {
            case 'Marker':
                M.MAP.pm.enableDraw('Marker', drawOptions);
                break;
            case 'Line':
                M.MAP.pm.enableDraw('Line', drawOptions);
                break;
            case 'Circle':
                M.MAP.pm.enableDraw('Circle', drawOptions);
                break;
            case 'Polygon':
                M.MAP.pm.enableDraw('Polygon', drawOptions);
                break;
            case 'CircleMarker':
                M.MAP.pm.enableDraw('CircleMarker', drawOptions);
                break;
            case 'Rectangle':
                M.MAP.pm.enableDraw('Rectangle', drawOptions);
                break;
            default:
                alert("Try to draw from code a type: " + this.type);
                throw new Error("Unknown type of draw");
        }
    }


    // #region Draw Info

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

    drawInfoGetHtml(id, info, initialHtml = [], endHtml = [], uavPickerType = 'none') {
        initialHtml.push(this._drawInfoGetValues(id));
        initialHtml.push(this._drawInfoGetHeight(id, info));
        if (uavPickerType != 'none') {
            initialHtml.push(this._drawInfoGetUavPicker(id, info, uavPickerType));
        }
        initialHtml.push(endHtml);
        return initialHtml;
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
        let heightMin = info.layer.pm.options.height[0];
        let heightMax = info.layer.pm.options.height[1];

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
        let info = myargs;
        info.drawManager.height = [args.heightMin, args.heightMax];
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
                drawManager.drawUserOptions.uavList = { uavName: true };
            }
        } else {
            drawManager.drawUserOptions.uavList[uavName] = true;
        }
    }

    //#endregion

}