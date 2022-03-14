class Polygon extends DrawManager
{
    constructor(name, options = undefined) {
        super('Polygon', name, options);
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

    addDrawInfo(htmlId, info, name = "Line", initialHtml = [], endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.drawManager.id;

        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
            let lngDict = HTMLUtils.addDict('input', `${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' });

            initialHtml.push(row);
        }

        return super.addDrawInfo(htmlId, info, name, initialHtml, endHtml, uavPickerType);
    }
}

class Area extends Polygon
{
    constructor(options = undefined) {
        super('Area', options);
    }

    addDrawInfo(id, info) {
        let name = 'Area';
        return super.addDrawInfo(id, info, name, undefined, undefined, 'checkbox');
    }
}

