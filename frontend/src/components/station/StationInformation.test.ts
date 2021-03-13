import StationInformation from '@/components/station/StationInformation.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

const wrapper = wrapWithVuetifyAndStore(StationInformation);

describe('When mounting station information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
