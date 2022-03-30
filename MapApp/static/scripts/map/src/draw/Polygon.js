class Polygon extends DrawManager {
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Polygon', name, parameters, options, layerOptions);
    }

    codeDraw(values, options = undefined, layerOptions = {}, missionId = undefined) {
        if (values.length < 1) {
            return;
        }
        if (missionId !== undefined) {
            layerOptions['color'] = M.MISSION_MANAGER.getColors(missionId)[1];
        }
        return super.codeDraw(values, options, layerOptions);
    }

    _addChangeCallback(id, info) {
        let htmlId = [];
        let nameId = [];
        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            htmlId.push(`${id}-${i}-lat`);
            htmlId.push(`${id}-${i}-lng`);

            nameId.push(`${i}-lat`);
            nameId.push(`${i}-lng`);
        }
        Utils.addFormCallback(`${id}-change`, htmlId, nameId, this._changeCallback.bind(this), id, info);
    }

    _changeCallback(myargs, inputs) {
        let layer = myargs[1].layer;

        let values = [];
        let len = Math.floor(Object.keys(inputs).length / 2);
        for (let i = 0; i < len; i++) {
            values.push([]);
        }
        for (var key in inputs) {
            let value = inputs[key];
            let keyParse = key.split('-');
            let index1 = parseInt(keyParse[0]);
            if (keyParse[1] == 'lat') {
                values[index1][0] = value;
            } else if (keyParse[1] == 'lng') {
                values[index1][1] = value;
            }
        }

        layer.setLatLngs(values);
    }

    drawInfoAdd(htmlId, info, name = "Line", initialHtml = [], endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
            let lngDict = HTMLUtils.addDict('input', `${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' });

            initialHtml.push(row);
        }

        return super.drawInfoAdd(htmlId, info, name, initialHtml, endHtml, uavPickerType);
    }
}

class Area extends Polygon {
    constructor(status, options = {}, layerOptions = undefined, parameters = config.Layers.Polygon.Area.parameters) {
        super(status, 'Area', parameters, options, layerOptions);
        this.configFile = config.Layers.Polygon.Area;
    }

    drawInfoAdd(htmlId, info) {
        let name = 'Area';
        return super.drawInfoAdd(htmlId, info, name, undefined, undefined, 'checkbox');
    }

    drawInfoInitialize(id, info) {
        Utils.addButtonsCallback(`${id}-Swarming-item`, this.clickAlgorithmsListCallback.bind(this), id, info);
        super.addParametersCallback(id, this.parameters, info);
        super.drawInfoInitialize(id, info);
    }

    clickAlgorithmsListCallback(e, args) {
        args[1].drawManager.options.algorithm = e.innerText;

        let button = document.getElementById(`${args[0]}-Swarming-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
    }

    // Replace add Parameters
    addParametersHtml(id, info) {
        let options = info.drawManager.options;
        let endHtml = [];
        endHtml.push(HTMLUtils.initDropDown(`${id}-Swarming`, this.configFile.algorithmList, this.configFile.algorithm));

        let paramHtml = super.getHtmlParameters(id, options, this.parameters);
        for (let i = 0; i < paramHtml.length; i++) {
            endHtml.push(paramHtml[i]);
        }

        return HTMLUtils.addDict('collapse', `${this.htmlId}-SwarmCollapse`, {}, 'Planner', true, endHtml);
    }
}
