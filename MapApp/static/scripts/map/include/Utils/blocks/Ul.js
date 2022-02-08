class Ul extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'list': args[0],
            }
        );
    } 

    static addHTML(content) {

        let label = document.createElement('UL');

        for (let i = 0; i < content.list.length; i++) {
            let div = document.createElement('DIV');
            div.setAttribute('class', 'row m-1');

            let listElement = document.createElement('LI');
            HTMLUtils.addHTML(listElement, content.list[i]);

            div.appendChild(listElement);
            label.appendChild(div);
        }
        return label;
    }
}