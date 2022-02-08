
class Label extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'text': args[0],
            }
        );
    } 

    static addHTML(content) {
        let label = document.createElement('label');
        label.innerHTML = content.text;
        return label;
    }
}