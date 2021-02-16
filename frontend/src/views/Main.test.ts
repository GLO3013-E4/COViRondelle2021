import Main from '@/views/Main.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

const wrapper = wrapWithVuetifyAndStore(Main);

describe('When mounting main view', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
