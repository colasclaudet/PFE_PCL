varying vec4 uv;
uniform sampler2D texture;
uniform vec4 color;

void main() {
      gl_FragColor =color; //texture2D(texture, uv.st);
}
