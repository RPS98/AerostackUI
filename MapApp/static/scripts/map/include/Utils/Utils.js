class HTMLUtils
{

    static addContent(htmlInfo) {
        let id = htmlInfo['container_id'];
        let content = htmlInfo['content'];

        let container = document.getElementById(id);
        
        for (let i=0; i<content.length; i++) {
            let htmlContent = null;
            let element = content[i];

            switch (element['type']) {
                case 'collapse':
                    break;
                case 'row':
                    break;
                case 'input':
                    break;
                default:
                    throw "Unknown type";
            }

            container.appendChild(content[i]);
        }
    }

    static addCollapse(id, name, content) {
        return {
            'id': id,
            'type': 'collapse',
            'content': {
                'name': name,
                'content': content
            }
        }
    }

    static addRow(content) {
        return {
            'id': id,
            'type': 'row',
            'content': content
        }
    }

    static addDict(id, type, content) {
        return {
            'id': id,
            'type': type,
            'content': content
        }
    }

    static addInputDict(id, type, defaultValue, class_field='') {
        return HTMLUtils.addDict(
            id,
            'input',
            {
                'inputType': type,
                'defaultValue': defaultValue,
                'class_field': class_field
            }
        );
    }

    static addCollapseGroup(groupId, groupName, groupContent) {
        /*
        <div class="d-grid gap-2 m-2">
            <button class="btn btn-secondary btn-collap-toggle" type="button" data-bs-toggle="collapse" data-bs-target="#collapse1">
                Go to
                <i class="icon-collap-toggle fas fa-plus"></i>
            </button>
            <div class="collapse multi-collapse" id="collapse1">
                <div class="row my-1 mx-1">
                </form>
                    <div class="col-4">
                    <input type="text" class="form-control" placeholder="latitude" required="required" id="map-view-lat">
                    </div>
                    <div class="col-4">
                    <input type="text" class="form-control" placeholder="longitude" required="required" id="map-view-long">
                    </div>
                    <button type="submit" class="col-4 btn btn-primary" id="map-view-btn">Go</button>
                <form>
                </div>
            </div>
        </div>
        */

        let div = document.createElement('div');
        let btn = document.createElement('button');
        let div_collapse = document.createElement('div');

        btn.setAttribute('class', 'btn btn-secondary btn-collap-toggle');
        btn.setAttribute('type', 'button');
        btn.setAttribute('data-bs-toggle', 'collapse');
        btn.setAttribute('data-bs-target', '#' + groupId);
        btn.innerHTML = `${groupName} <i class="icon-collap-toggle fas fa-plus"></i>`;

        div_collapse.setAttribute('class', 'collapse multi-collapse');
        div_collapse.setAttribute('id', groupId);
        div_collapse.appendChild(groupContent);

        div.setAttribute('class', 'd-grid gap-2 m-2');
        div.appendChild(btn);
        div.appendChild(div_collapse);
    }

    static addRow(input = []) {
        let len = input.length;
        let div = document.createElement('div');
        div.setAttribute('class', 'row my-1 mx-1');

        let col = 12 / len;

        for (let i = 0; i < len; i++) {
            let div_col = document.createElement('div');
            div_col.setAttribute('class', 'col-' + col);
            div_col.appendChild(input[i]);
            div.appendChild(div_col);
        }
    }

    static addLabel(text) {
        let label = document.createElement('label');
        label.innerHTML = text;
        return label;
    }

    static addInput(type, id, defaultValue, class_field='') {
        let input = document.createElement('input');
        input.setAttribute('type', type);
        input.setAttribute('id', id);
        input.setAttribute('value', defaultValue);
        input.setAttribute('class', class_field);
        return input;
    }

    static addOneInputParam(name, defaultValue, paramType, paramId) {
        let label = HTMLUtils.addLabel(name);
        let input = HTMLUtils.addInput(paramType, paramId, defaultValue);
        HTMLUtils.addRow([label, input]);
    }
}



class Utils 
{
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
    static addButtonCallback(button_id, callback=Utils._nullFunct, args=[]) {
        // get button element
        const btn = document.getElementById(button_id);

        // add an event listener to the button
        btn.addEventListener('click', (e) => {
            // disable the refresh on the page when submit
            e.preventDefault(); 

            // call the button callback
            callback(args);
        });
    }
 
     /**
      * Add a listener for number inputs by a button
      * @param {string}   button_id - id of the button
      * @param {string}   inputs_id - list of form inputs id
      * @param {string}   names_id  - name of elements in the list
      * @param {function} callback  - callback when press button
      * @param {list}     args      - arguments list pass to callback
      */ 
    static addFormCallback(button_id, inputs_id, names_id, callback=Utils._nullFunct, args=[]) {

        // get button element
        const btn = document.getElementById(button_id);

        // check if inputs and names has the same length
        if (inputs_id.length != names_id.length) {
            throw "Input id and names id lengths are not equals";
        }

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
    }

    static setColor(map, UAV_color) 
    {
        map.pm.setPathOptions({color: UAV_color});
    }

    static onlyUnique(value, index, self) {
        return self.indexOf(value) === index;
    }
      
 }