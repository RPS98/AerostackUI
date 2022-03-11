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

    _uavPickerCallback(uavName, value, userargs) {

        let type = userargs[0];
        let layer = userargs[1];
        let drawManager = layer.pm.options.drawManager;

        if (type == 'radio') {
            if (value) {
                drawManager.drawUserOptions.uavList = { uavName: true };
            }
        } else {
            drawManager.drawUserOptions.uavList[uavName] = true;
        }
    }

    getHtmlDrawInfo(htmlId, layer, name = "Marker", htmlValues = [], htmlCode = [], addUavPicker = false, uavPickerType = 'checkbox') {
        let id = layer.pm.options.drawManager.id;

        let heightMin = layer.pm.options.height[0];
        let heightMax = layer.pm.options.height[1];

        // Values HTML
        let change = HTMLUtils.addDict('button', `${htmlId}-${id}-change`, { 'class': 'btn btn-primary' }, 'Change');
        let remove = HTMLUtils.addDict('button', `${htmlId}-${id}-remove`, { 'class': 'btn btn-danger' }, 'Remove');
        let changeDiv = HTMLUtils.addDict('div', `none`, {}, [change]);
        let removeDiv = HTMLUtils.addDict('div', `none`, {}, [remove]);
        let layerRow = HTMLUtils.addDict('div', `none`, { 'class': 'btn-group d-flex justify-content-evenly', 'role': 'group' }, [changeDiv, removeDiv]);

        // Height range HTML
        let heightInputMin = HTMLUtils.addDict('input', `${htmlId}-heightInputMin`, { 'class': 'form-control', 'required': 'required', 'value': heightMin }, 'text', heightMin);
        let heightInputMax = HTMLUtils.addDict('input', `${htmlId}-heightInputMax`, { 'class': 'form-control', 'required': 'required', 'value': heightMax }, 'text', heightMax);
        let heightRangeBtn = HTMLUtils.addDict('button', `${htmlId}-heighRangeBtn`, { 'class': 'btn btn-primary' }, 'Set Height (m)');

        let heightInputMinDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [heightInputMin]);
        let heightInputMaxDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [heightInputMax]);
        let heightRangeBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [heightRangeBtn]);

        let heightRangeRow = HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [heightInputMinDiv, heightInputMaxDiv, heightRangeBtnDiv]);

        let uavPickerListCollapse = [];
        if (addUavPicker) {
            let list = [['auto', true]];
            let uavPickerList = M.getUavPickerDict(uavPickerType, `${htmlId}-UAVPicker`, list, this._uavPickerCallback.bind(this), uavPickerType, layer);
            uavPickerListCollapse = HTMLUtils.addDict('collapse', `${htmlId}-UAVCollapse`, {}, 'UAV Picker', true, [uavPickerList]);
        }

        return HTMLUtils.addDict('collapse', `${htmlId}-${id}-Collapse`, {}, `${name} ${id}`, false, [htmlValues, layerRow, heightRangeRow, uavPickerListCollapse, htmlCode]);
    }

    getHtmlCodeInfo() {
        throw new Error("Method not implemented.");
    }

    getLayerInfo() {
        throw new Error("Method not implemented.");
    }
}